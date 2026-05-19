"""
i2head_facs_node.py

FACS (Facial Action Coding System) -pohjainen ilmeiden hallinta InMoov i2Headille.

Vastaanottaa:
  /facs/au       (interface/msg/FACS_au)        – yksittäisen AU:n aktivointi
  /facs/expression (std_msgs/String)            – nimetty ilme

Julkaisee:
  /i2head/joint_commands (JointTrajectory)       – bridge-nodelle

AU-blendaus: summaa aktiivisten AU:iden vaikutukset additiivisesti.
Samaa prioriteettia olevat AU:t blendautuvat; korkeampi prioriteetti ylikirjoittaa.

Usage:
  python3 client/i2head_facs_node.py --ros-args \
    -p facs_config:=config/i2head_facs.yaml
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from interface.msg import FACS_au
import yaml
import os
import math
import random
from typing import Dict, List, Optional


class ActiveAU:
    """Tracks a single AU's current activation state."""

    def __init__(self, au_id: int, intensity: float, priority: int, duration_ns: int):
        self.au_id = au_id
        self.intensity = intensity
        self.priority = priority
        self.duration_ns = duration_ns


class I2HeadFACSNode(Node):

    def __init__(self):
        super().__init__("i2head_facs")

        self.declare_parameter("facs_config", "")
        self.declare_parameter("au_topic", "/facs/au")
        self.declare_parameter("expression_topic", "/facs/expression")
        self.declare_parameter("command_topic", "/i2head/joint_commands")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("neutral_after_sec", 3.0)
        self.declare_parameter("idle_expression_chance", 0.0)
        self.declare_parameter("idle_interval_sec", 8.0)

        config_path = self.get_parameter("facs_config").value
        au_topic = self.get_parameter("au_topic").value
        expr_topic = self.get_parameter("expression_topic").value
        cmd_topic = self.get_parameter("command_topic").value
        rate = self.get_parameter("publish_rate").value
        self.neutral_after_sec = self.get_parameter("neutral_after_sec").value
        self.idle_chance = self.get_parameter("idle_expression_chance").value
        self.idle_interval = self.get_parameter("idle_interval_sec").value

        # AU definitions: au_id -> {servo_name: {neutral, activated}}
        self.au_defs: Dict[int, dict] = {}
        # Expression definitions: name -> {aus: {au_id: intensity}, duration, sequence}
        self.expr_defs: Dict[str, dict] = {}
        # Active AUs: au_id -> ActiveAU
        self.active_aus: Dict[int, ActiveAU] = {}
        # Last expression publish time (for neutral timer)
        self.last_expression_time = 0.0

        self._load_config(config_path)

        self.cmd_pub = self.create_publisher(JointTrajectory, cmd_topic, 10)
        self.create_subscription(FACS_au, au_topic, self.au_callback, 10)
        self.create_subscription(String, expr_topic, self.expression_callback, 10)

        # Timer for idle expressions
        if self.idle_chance > 0:
            self.create_timer(self.idle_interval, self._idle_timer_callback)

        # Servo angle cache for damped transitions
        self.current_angles: Dict[str, float] = {}
        self._neutral_timer = None
        self._seq_timer = None
        self._seq_steps = []
        self._seq_index = 0
        self._seq_name = ""

        self.get_logger().info(
            f"FACS node ready. {len(self.au_defs)} AUs, {len(self.expr_defs)} expressions"
        )

    def _load_config(self, path: str):
        if not path or not os.path.isfile(path):
            self.get_logger().warn("No FACS config. Use facs_config parameter.")
            return

        with open(path) as f:
            data = yaml.safe_load(f)

        facs = data.get("facs", {})

        # Load AU definitions
        raw_aus = facs.get("action_units", {})
        for key, cfg in raw_aus.items():
            if key.startswith("au_"):
                au_id = int(key[3:])
                self.au_defs[au_id] = cfg
            else:
                self.get_logger().warn(f"Skipping unknown AU key: {key}")

        # Load expression definitions
        raw_exprs = facs.get("expressions", {})
        for name, cfg in raw_exprs.items():
            self.expr_defs[name] = cfg

        self.get_logger().info(
            f"Loaded {len(self.au_defs)} AUs, {len(self.expr_defs)} expressions"
        )

    def au_callback(self, msg: FACS_au):
        if msg.au_id not in self.au_defs:
            self.get_logger().warn(f"Unknown AU: {msg.au_id}")
            return

        self.get_logger().info(
            f"AU {msg.au_id}: intensity={msg.intensity:.2f}, priority={msg.priority}"
        )

        if msg.intensity <= 0.0:
            self.active_aus.pop(msg.au_id, None)
        else:
            self.active_aus[msg.au_id] = ActiveAU(
                msg.au_id, min(1.0, msg.intensity), msg.priority, msg.duration_ns
            )

        self._blend_and_publish()

    def expression_callback(self, msg: String):
        name = msg.data.strip().lower()
        if name not in self.expr_defs:
            self.get_logger().warn(f"Unknown expression: '{name}'")
            return

        self.get_logger().info(f"Expression: {name}")
        expr = self.expr_defs[name]

        if "sequence" in expr:
            self._execute_sequence(name, expr["sequence"])
        else:
            self._apply_expression_aus(expr.get("aus", {}), expr.get("duration", 0.3))
            self._schedule_neutral()

    def _apply_expression_aus(self, au_map: Dict[int, float], duration_sec: float):
        """Set all AUs for an expression, clearing previous AUs first."""
        self.active_aus.clear()
        duration_ns = int(duration_sec * 1e9)
        for au_id, intensity in au_map.items():
            if au_id in self.au_defs and intensity > 0:
                self.active_aus[au_id] = ActiveAU(au_id, intensity, 2, duration_ns)

        self._blend_and_publish()

    def _execute_sequence(self, name: str, steps: list):
        """Execute a multi-step expression (e.g., blink = close + open)."""
        self._seq_timer = None
        self._seq_name = name
        self._seq_steps = list(steps)
        self._seq_index = 0
        self._execute_seq_step()

    def _execute_seq_step(self):
        if self._seq_index >= len(self._seq_steps):
            self.get_logger().info(f"Sequence '{self._seq_name}' complete")
            self._schedule_neutral()
            return

        step = self._seq_steps[self._seq_index]
        dur = step.get("duration", 0.15)
        self._apply_expression_aus(step.get("aus", {}), dur)
        self._seq_index += 1

        if self._seq_index < len(self._seq_steps):
            if self._seq_timer:
                self._seq_timer.cancel()
            self._seq_timer = self.create_timer(dur + 0.05, self._execute_seq_step)

    def _blend_and_publish(self):
        """Blend all active AUs into servo angles and publish JointTrajectory."""
        if not self.active_aus:
            return

        # Group AUs by priority
        priority_groups: Dict[int, List[ActiveAU]] = {}
        for au in self.active_aus.values():
            priority_groups.setdefault(au.priority, []).append(au)

        # Use only highest priority level with active AUs
        top_priority = max(priority_groups.keys())

        # Within top priority, blend additively starting from neutral
        blended: Dict[str, float] = {}

        def get_base(name):
            return blended.get(name, 90.0)

        for au in priority_groups[top_priority]:
            cfg = self.au_defs.get(au.au_id)
            if not cfg:
                continue
            for srv, scfg in cfg.get("servos", {}).items():
                neutral = scfg.get("neutral", 90.0)
                activated = scfg.get("activated", neutral)
                if srv not in blended:
                    blended[srv] = neutral
                blended[srv] += (activated - neutral) * au.intensity

        # Dampen transitions
        for name in list(blended.keys()):
            current = self.current_angles.get(name, blended[name])
            diff = blended[name] - current
            blended[name] = current + diff * 0.5
            self.current_angles[name] = blended[name]

        if not blended:
            return

        msg = JointTrajectory()
        msg.joint_names = list(blended.keys())
        point = JointTrajectoryPoint()
        point.positions = [blended[j] for j in msg.joint_names]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        msg.points.append(point)
        self.cmd_pub.publish(msg)

    def _schedule_neutral(self):
        self.last_expression_time = self.get_clock().now().nanoseconds / 1e9
        if self.neutral_after_sec <= 0:
            return
        if self._neutral_timer:
            self._neutral_timer.cancel()
        self._neutral_timer = self.create_timer(self.neutral_after_sec, self._return_to_neutral)

    def _return_to_neutral(self):
        if self._neutral_timer:
            self._neutral_timer.cancel()
            self._neutral_timer = None
        self.active_aus.clear()
        self._blend_and_publish()

    def _idle_timer_callback(self):
        """Occasionally trigger a random expression during idle."""
        if self.active_aus:
            return
        if random.random() >= self.idle_chance:
            return

        candidates = [n for n in self.expr_defs if n not in ("neutral", "blink")]
        if not candidates:
            return

        chosen = random.choice(candidates)
        self.get_logger().info(f"Idle expression: {chosen}")
        msg = String()
        msg.data = chosen
        self.expression_callback(msg)


def main(args=None):
    rclpy.init(args=args)
    node = I2HeadFACSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
