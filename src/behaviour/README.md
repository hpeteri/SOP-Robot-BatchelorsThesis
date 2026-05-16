# behaviour

Robotin käyttäytymispaketti. Päättää miten robotti reagoi perception-paketilta saatuun dataan.

## FaceTrackNode

Muuntaa tunnistettujen kasvojen sijainnin liikekomentoiksi.

**Tilaajat:**
- `/faces` (`face_tracker_msgs/Faces`) – tunnistetut kasvot

**Julkaisijat:**
- `/head_move_goal` (`interface/HeadMovementGoal`) – liikekomento pään ja silmien ohjaukseen

**Toiminta:**
1. Valitsee seurattavan kasvon (suurin bounding box)
2. Laskee kulmaeron kasvon ja kameran keskipisteen välillä
3. Jos kulmaero > `head_movement_threshold`: lähettää pään liikekäskyn
4. Muuten: lähettää silmien liikekäskyn tarkkaan seurantaan

### ROS2-parametrit

| Parametri | Tyyppi | Oletus | Kuvaus |
|-----------|--------|--------|--------|
| `camera_diagonal_fov` | double | 1.19555 | Kameran diagonaalinen FOV (rad) |
| `camera_resolution_x` | int | 1280 | Kameran leveys (px) |
| `camera_resolution_y` | int | 960 | Kameran korkeus (px) |
| `coeff_head_pan` | double | -1.04387 | Kamerakulman ja head_pan-servon välinen kerroin |
| `priority_face_track` | int | 2 | Kasvojenseurannan prioriteetti |
| `tracking_goal_min_interval` | double | 0.1 | Minimi aika liikekomentojen välillä (s) |
| `head_movement_threshold` | double | 0.15 | Kulmakynnys pään liikkeelle (rad) |

## HeadMovementNode

Vastaanottaa liikekomentoja ja ohjaa robotin päätä ja silmiä `FollowJointTrajectory`-actioneilla.

**Tilaajat:**
- `/head_move_goal` (`interface/HeadMovementGoal`) – liikekomento
- `/head_gesture_command` (`std_msgs/String`) – elekomennot (`nod`, `shake`)
- `/head_controller/controller_state` – pään nivelten tila
- `/eyes_controller/controller_state` – silmien nivelten tila

**Action-clientit:**
- `/head_controller/follow_joint_trajectory`
- `/eyes_controller/follow_joint_trajectory`

**Prioriteettipohjainen välitys:**
- 3 = GESTURE (elect, korkein prioriteetti)
- 2 = FACE_TRACK (kasvojenseuranta)
- 1 = IDLE (matalin prioriteetti)

### ROS2-parametrit

| Parametri | Tyyppi | Oletus | Kuvaus |
|-----------|--------|--------|--------|
| `priority_idle` | int | 1 | Idle-prioriteetti |
| `priority_face_track` | int | 2 | Track-prioriteetti |
| `priority_gesture` | int | 3 | Ele-prioriteetti |
| `coeff_head_pan` | double | -1.04387 | head_pan-kerroin |
| `coeff_head_pitch` | double | -2.67659 | head_pitch-kerroin |
| `coeff_eye_horizontal` | double | -2.67659 | eye_horizontal-kerroin |
| `coeff_eye_vertical` | double | 4.01489 | eye_vertical-kerroin |
| `head_pan_min_rad` | double | -0.7 | Pään kierto minimi (rad) |
| `head_pan_max_rad` | double | 0.7 | Pään kierto maksimi (rad) |
| `head_pitch_min_rad` | double | -0.3 | Pään kallistus minimi (rad) |
| `head_pitch_max_rad` | double | 0.3 | Pään kallistus maksimi (rad) |
| `eye_h_min_rad` | double | -0.2 | Silmän vaaka minimi (rad) |
| `eye_h_max_rad` | double | 0.2 | Silmän vaaka maksimi (rad) |
| `eye_v_min_rad` | double | -0.15 | Silmän pysty minimi (rad) |
| `eye_v_max_rad` | double | 0.15 | Silmän pysty maksimi (rad) |
| `head_joint_names` | string[] | [head_pan_joint, head_pitch_joint] | Pään nivelten nimet |
| `eye_joint_names` | string[] | [eye_horizontal_joint, eye_vertical_joint] | Silmien nivelten nimet |
| `default_trajectory_duration` | double | 0.3 | Liikeradan oletuskesto (s) |

## idle

(Tulossa) Satunnaisliikkeet ja idlen aikainen käyttäytyminen.

## Huomioitavaa

- Elekt ovat blokkaavia (time.sleep) – ei-ideaali, mutta toimiva
- idle-alipaketti on varattu tulevalle satunnaisliikkeelle
