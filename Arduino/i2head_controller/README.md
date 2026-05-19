# i2Head Controller

Data-driven PCA9685 servo controller for InMoov i2Head facial expressions.

## Architecture

```
YAML config (config/i2head.yaml)
    |  "source of truth": maps joints -> channels, pulse ranges, limits
    v
ROS2 bridge (client/i2head_bridge_node.py)
    |  loads YAML, sends CFG commands to Arduino at startup
    |  subscribes to /i2head/joint_commands
    v
Arduino firmware (i2head_controller.ino)
    |  stores config in runtime arrays
    |  applies limits, pulse mapping, direction inversion
    v
PCA9685 (I2C) -> RC servos
```

## Data-Driven Design

**All servo-specific parameters come from YAML:**
- Channel assignment (which servo on which PCA9685 output)
- Pulse range (min/max microseconds for 0/180 degrees)
- Angle limits (mechanical range of each joint)
- Home position (neutral/starting angle)
- Direction (reversed for mirrored joints)

**Firmware is completely generic:**
- Same binary works for any head module
- Configuration received at startup via `CFG:` serial commands
- No hardcoded servo values - everything is runtime data

## Protocol

| Command | Format | Description |
|---------|--------|-------------|
| Config | `CFG:ch:minP:maxP:minA:maxA:home:rev` | Configure one channel |
| Config done | `CFG_DONE` | Apply all configs, move to home |
| Home | `HOME` | Move all servos to home positions |
| Move | `ch:ang,ch:ang,...` | Set channels to angles |

## Wiring

- PCA9685 SCL -> Arduino SCL (A5 on Uno, 21 on Mega)
- PCA9685 SDA -> Arduino SDA (A4 on Uno, 20 on Mega)
- PCA9685 VCC -> 5V, GND -> GND
- Servos on PCA9685 channels 0-15

## Usage

```bash
# Load config and start bridge
ros2 run i2head_bridge i2head_bridge_node \
  --ros-args -p joint_config:=config/i2head.yaml \
  -p serial_port:=/dev/ttyACM0

# Interactive testing
python3 client/i2head_tester.py /dev/ttyACM0 115200 config/i2head.yaml
```
