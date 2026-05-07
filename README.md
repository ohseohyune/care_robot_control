# care_robot_control — ROS2 Jazzy

목 모터(Dynamixel XC330) 제어 및 IMU 데이터 브리지 패키지.  
사람 추종 알고리즘은 `seniorcare_robot/master_node`의 behavior tree(`TrackingPerson`)가 담당

---

## 패키지 구조

```
care_robot_control/
├── care_robot_control/
│   ├── neck_controller_node.py   # Dynamixel XC330 목 모터 제어
│   └── imu_bridge_node.py        # ebimu IMU 데이터 → master_node 중계
└── launch/
    └── real_follow.launch.py     # 하드웨어 실행 launch
```

---

## 시스템 아키텍처

```
[ebimu_original]
      │ /bridge (humanoid_interfaces/ImuMsg)
      ▼
[imu_bridge_node] ──── Imu (senior_msg/ImuMsg) ────▶ [master_node]
                                                             │
[neck_controller_node] ─── /neck_yaw_state (Float64) ──────▶│
      ▲                                                      │
      │ /neck_yaw_target (Float64)                           │ /base_command
      │                                              (senior_msg/Master2Base)
      └──────────────── master_node이 퍼블리시 ──────▶ [capstone_base_driver]
```

- **master_node** (`seniorcare_robot`): 사람 추종 판단, 베이스 모터 명령 발행
- **neck_controller_node**: `/neck_yaw_target` 수신 → Dynamixel 목 모터 구동, `/neck_yaw_state` 퍼블리시
- **imu_bridge_node**: ebimu 노드의 `/bridge` 토픽을 master_node가 구독하는 `Imu` 토픽으로 변환

---

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select care_robot_control
source install/setup.bash
```

---

## 실행

### 전체 런치 (목 모터 + IMU 브리지 + 베이스 드라이버)

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch care_robot_control real_follow.launch.py \
  device_name:=/dev/ttyUSB0 \
  baud_rate:=57600 \
  dxl_id:=1 \
  center_tick:=2048
```

> master_node는 별도 터미널에서 실행:
> ```bash
> ros2 run seniorcare_robot master_node
> ```

---

### 목 모터 단독 테스트

```bash
ros2 run care_robot_control neck_controller \
  --ros-args \
  -p simulation_mode:=false \
  -p device_name:=/dev/ttyUSB0 \
  -p baud_rate:=57600 \
  -p dxl_id:=1 \
  -p center_tick:=2048
```

다른 터미널에서 목표 각도 전송 (단위: rad):

```bash
# 정면 (0°)
ros2 topic pub /neck_yaw_target std_msgs/msg/Float64 "data: 0.0" --once

# 왼쪽 30° (0.52 rad)
ros2 topic pub /neck_yaw_target std_msgs/msg/Float64 "data: 0.52" --once

# 오른쪽 30°
ros2 topic pub /neck_yaw_target std_msgs/msg/Float64 "data: -0.52" --once

# 왼쪽 최대 60° (1.047 rad)
ros2 topic pub /neck_yaw_target std_msgs/msg/Float64 "data: 1.047" --once
```

현재 목 각도 확인:

```bash
ros2 topic echo /neck_yaw_state
```

---

## 퍼블리시/구독 토픽 요약

| 노드 | 방향 | 토픽 | 타입 | 설명 |
|------|------|-------|------|------|
| neck_controller_node | Subscribe | `/neck_yaw_target` | `std_msgs/Float64` | 목표 목 각도 [rad] |
| neck_controller_node | Publish | `/neck_yaw_state` | `std_msgs/Float64` | 현재 목 각도 [rad] |
| imu_bridge_node | Subscribe | `/bridge` | `humanoid_interfaces/ImuMsg` | ebimu 원본 데이터 |
| imu_bridge_node | Publish | `Imu` | `senior_msg/ImuMsg` | master_node로 전달 |

---

## 주요 파라미터 (neck_controller_node)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `device_name` | `/dev/ttyUSB0` | U2D2 포트 경로 |
| `baud_rate` | `57600` | Dynamixel baud rate |
| `dxl_id` | `1` | Dynamixel 모터 ID |
| `center_tick` | `2048` | 정면(0°) 틱 값 |
| `reverse_direction` | `false` | 방향 반전 여부 |
| `neck_limit` | `1.047` | 최대 회전각 [rad] (±60°) |
| `publish_hz` | `50.0` | 상태 퍼블리시 주기 [Hz] |
| `profile_velocity_rpm` | `25.0` | 모터 이동 속도 [RPM] |
| `command_timeout` | `0.5` | 명령 수신 타임아웃 [s] |
| `park_on_shutdown` | `true` | 종료 시 정면 복귀 여부 |

---

## 사전 준비 (최초 1회)

### 1. dynamixel-sdk 설치

```bash
pip install dynamixel-sdk
```

### 2. USB 포트 권한 설정

```bash
# 매번 설정
sudo chmod 666 /dev/ttyUSB0

# 영구 설정 (로그아웃 후 재로그인 필요)
sudo usermod -aG dialout $USER
```

### 3. 포트 및 모터 ID 확인

```bash
ls /dev/ttyUSB*
```

모터 ID와 baud rate는 **Dynamixel Wizard 2.0** 으로 미리 확인한다.  
기본값: ID=1, baud=57600

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `Failed to open Dynamixel port` | 포트 권한 없음 | `sudo chmod 666 /dev/ttyUSB0` |
| `Missing dynamixel_sdk` | SDK 미설치 | `pip install dynamixel-sdk` |
| `Could not read Present Position` | ID/baud 불일치 | Dynamixel Wizard로 확인 |
| `Dynamixel hardware is not ready` | 전원 미공급 또는 케이블 불량 | 모터 전원·TTL 케이블 점검 |
| 목이 반대 방향으로 움직임 | 방향 설정 오류 | `reverse_direction:=true` 추가 |
| 목 시작 위치가 정면이 아님 | center_tick 오설정 | Wizard로 정면 틱 확인 후 `center_tick` 조정 |
| `Imu` 토픽에 데이터 없음 | ebimu 노드 미실행 | `ros2 run ebimu_original e2box_imu` 실행 확인 |

---

## 주의사항

- XC330은 TTL Half-Duplex 통신 → 일반 USB-Serial 불가, **U2D2** 필수
- 종료 시 자동으로 목이 정면(0°)으로 복귀 후 토크 OFF (`park_on_shutdown=true`)
- 비전 각도 기준은 **neck_yaw_link 프레임** 기준
