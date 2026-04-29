# Care Robot Control — ROS2 Jazzy

## 패키지 구조

```
ros2_ws/
└── src/
    └── care_robot_control/
        ├── care_robot_control/
        │   ├── neck_controller_node.py    # XC330 Dynamixel 제어 (실제 HW / 시뮬)
        │   ├── person_follower_node.py    # 사람 추종 제어 노드
        │   ├── robot_simulator_node.py    # 유니사이클 시뮬레이터 + 비전 mock
        │   └── visualizer_node.py         # matplotlib 실시간 시각화
        └── launch/
            ├── sim_launch.py              # 시뮬레이션 전용 launch
            └── real_follow.launch.py      # 실제 하드웨어 launch
```

---

## 시스템 아키텍처

```
[robot_simulator_node]──────/person_detection_raw──────▶[person_follower_node]
      ▲                                                         │
      │ /cmd_vel                                           /neck_yaw_target
      │ /neck_yaw_state                                         │
      │                                                         ▼
      └─────────────────────────────────────────────[neck_controller_node]
                                                        (simulation_mode=True)

실제 HW 시: robot_simulator_node 제거, 비전 모듈이 /person_detection 퍼블리시
```

---

## 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select care_robot_control
source install/setup.bash
```

---

## 시뮬레이션 실행

```bash
# 기본 (원형 궤도)
ros2 launch care_robot_control sim_launch.py

# 직선 이동 시나리오
ros2 launch care_robot_control sim_launch.py person_mode:=linear person_speed:=0.5

# 정지 상태
ros2 launch care_robot_control sim_launch.py person_mode:=still
```

### 시각화 (별도 터미널)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run care_robot_control visualizer
```

### 상태 모니터링

```bash
ros2 topic echo /robot_state
ros2 topic hz /cmd_vel
ros2 topic echo /person_detection_raw
```

---

## 실제 Dynamixel(목 모터) 실행

### 사전 준비

#### 1. dynamixel-sdk 설치

```bash
pip install dynamixel-sdk
```

#### 2. USB 포트 권한 설정

```bash
# 매번 설정하는 방법
sudo chmod 666 /dev/ttyUSB0

# 영구 설정 (재부팅 후에도 유지)
sudo usermod -aG dialout $USER
# → 로그아웃 후 재로그인 필요
```

#### 3. 포트 확인 (U2D2 연결 후)

```bash
ls /dev/ttyUSB*
# 예: /dev/ttyUSB0 이 보이면 정상
```

#### 4. Dynamixel ID / 보드레이트 확인

Dynamixel Wizard 2.0 으로 모터 ID와 baud rate를 미리 확인한다.  
기본값: ID=1, baud=57600

---

### 실행 — 목 모터만 단독 구동

목 모터 노드만 켜고 수동으로 목표 각도를 보내 테스트할 때 사용한다.

```bash
source ~/ros2_ws/install/setup.bash

ros2 run care_robot_control neck_controller \
  --ros-args \
  -p simulation_mode:=false \
  -p device_name:=/dev/ttyUSB0 \
  -p baud_rate:=57600 \
  -p dxl_id:=1 \
  -p center_tick:=2048
```

다른 터미널에서 목표 각도 퍼블리시 (단위: rad):

```bash
# 정면(0도)
ros2 topic pub /neck_yaw_target std_msgs/Float64 "data: 0.0" --once

# 왼쪽 30도 (약 0.52 rad)
ros2 topic pub /neck_yaw_target std_msgs/Float64 "data: 0.52" --once

# 오른쪽 30도
ros2 topic pub /neck_yaw_target std_msgs/Float64 "data: -0.52" --once
```

현재 목 각도 확인:

```bash
ros2 topic echo /neck_yaw_state
```

---

### 실행 — 실제 하드웨어 전체 (목 모터 + 사람 추종)

비전 노드가 `/person_detection` 또는 `/person_detection_raw`를 퍼블리시하는 상태에서 실행한다.

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch care_robot_control real_follow.launch.py \
  device_name:=/dev/ttyUSB0 \
  baud_rate:=57600 \
  dxl_id:=1 \
  center_tick:=2048
```

# 테스트용 예시: /person_detection을 직접 퍼블리시
# 실제 비전 노드가 없을 때는 아래 예시로 사람 감지 데이터를 보낼 수 있다.
```bash
ros2 topic pub /person_detection care_robot_msgs/msg/PersonDetection \
  "{distance: 1.0, angle: 0.0, detected: true}" -r 5
```

# 또는 fallback으로 /person_detection_raw를 직접 퍼블리시
```bash
ros2 topic pub /person_detection_raw std_msgs/Float64MultiArray \
  "{data: [1.0, 0.0, 1.0]}" -r 5
```

#### 주요 파라미터 옵션

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `device_name` | `/dev/ttyUSB0` | U2D2 포트 경로 |
| `baud_rate` | `57600` | Dynamixel baud rate |
| `dxl_id` | `1` | Dynamixel 모터 ID |
| `center_tick` | `2048` | 정면(0°) 틱 값 |
| `reverse_direction` | `false` | 방향 반전 여부 |
| `neck_limit` | `1.047` | 최대 회전 각도 (rad, ±60°) |
| `profile_velocity_rpm` | `25.0` | 목 이동 속도 (RPM) |
| `v_max` | `0.6` | 최대 전진 속도 (m/s) |
| `omega_max` | `1.0` | 최대 회전 속도 (rad/s) |

#### 예: 포트가 ttyUSB1이거나 모터 ID가 다를 때

```bash
ros2 launch care_robot_control real_follow.launch.py \
  device_name:=/dev/ttyUSB1 \
  dxl_id:=2 \
  center_tick:=2048
```

---

## 트러블슈팅

| 증상 | 원인 | 해결 |
|------|------|------|
| `Failed to open Dynamixel port` | 포트 권한 없음 | `sudo chmod 666 /dev/ttyUSB0` |
| `Missing dynamixel_sdk` | SDK 미설치 | `pip install dynamixel-sdk` |
| `Could not read Present Position` | ID/baud 불일치 | Dynamixel Wizard로 ID·baud 확인 |
| `Dynamixel hardware is not ready` | 전원 미공급 또는 케이블 불량 | 모터 전원·TTL 케이블 점검 |
| 목이 반대 방향으로 움직임 | 방향 설정 오류 | `reverse_direction:=true` 추가 |
| 목이 정면이 아닌 위치에서 시작 | center_tick 오설정 | Wizard로 정면 틱 확인 후 `center_tick` 조정 |

---

## 핵심 파라미터 (URDF 기반)

| 항목 | 값 | 출처 |
|------|-----|------|
| 바퀴 반경 | 0.0325 m | URDF `wheel_radius` |
| 휠베이스 | 0.2115 m | URDF `left_wheel_y * 2` |
| 카메라 FOV (수평) | ~120° (2.094 rad) | ArduCam IMX708 spec |
| 목표 거리 | 1.0 m ± 0.1 m | 요구사항 |
| 제어 루프 | 50 Hz | |

---

## 주의사항

- XC330은 TTL Half-Duplex → 일반 USB-Serial 어댑터 불가, **U2D2** 또는 **USB2Dynamixel** 필요
- 비전 각도 기준이 **neck_yaw_link 프레임**인지 **base_link 프레임**인지 반드시 확인
- 종료 시 자동으로 목이 정면(0°)으로 복귀 후 토크 OFF (`park_on_shutdown=True`)
- 실내 운용 속도이므로 v_max = 0.6 m/s 적용 (JGB37-520 최대 0.85 m/s 대비 여유)

---
[확실히 돌아감]

 [TERMINAL 1]
 ~/ros2_ws/run_neck.sh

 [TERMINAL 2]
  # 오른쪽 30도
  ros2 topic pub /neck_yaw_target
  std_msgs/msg/Float64 "data: -0.52" -r 10
  

  # 왼쪽 60도 (최대)
  ros2 topic pub /neck_yaw_target
  std_msgs/msg/Float64 "data: 1.047" -r
   10

  # 오른쪽 60도 (최대)
  ros2 topic pub /neck_yaw_target
  std_msgs/msg/Float64 "data: -1.047"
  -r 10

  # 정면
  ros2 topic pub /neck_yaw_target
  std_msgs/msg/Float64 "data: 0.0" -r
  10

  현재 목 위치는 터미널 3에서 확인할 수
   있습니다:
  source ~/ros2_ws/install/setup.bash
  ros2 topic echo /neck_yaw_state


--
[FAKE PERSON info publish]

[ terminal 1 ]

●source ~/ros2_ws/install/setup.bash                                                                             
ros2 launch care_robot_control real_follow.launch.py device_name:=/dev/ttyUSB0 baud_rate:=57600 dxl_id:=1 center_tick:=2048

[ terminal 2 ]
ros2 topic pub /person_detection_raw std_msgs/msg/Float64MultiArray '{data: [1.0, 0.3, 1.0]}' -r 15 

---
[dynamixel motor & visualizer's neck motor motion syncyronized]

  터미널 1 — 실제 하드웨어:
  ros2 launch care_robot_control real_follow.launch.py device_name:=/dev/ttyUSB0 baud_rate:=57600 dxl_id:=1
  center_tick:=2048

  터미널 2 — 가짜 감지 데이터:
  ros2 topic pub /person_detection_raw std_msgs/msg/Float64MultiArray '{data: [1.0, 0.3, 1.0]}' -r 15

  # 좌우로 왔다갔다 (기본, 목이 따라 움직임)                                                                      
  python3 ~/ros2_ws/fake_person.py sweep                                                                          
                                                                                                                 
  # 거리+각도 동시 변화                                                                                           
  python3 ~/ros2_ws/fake_person.py circle                   
                                                                                                                  
  # 정면에서 다가왔다 멀어졌다
  python3 ~/ros2_ws/fake_person.py approach   
  # random
  python3 ~/ros2_ws/fake_person.py random_walk
  python3 ~/ros2_ws/fake_person.py chaos 

  ┌─────────────┬───────────────────────────────────────────┐                                                     
  │    모드     │                  움직임                   │
  ├─────────────┼───────────────────────────────────────────┤                                                     
  │ random_walk │ 랜덤하게 각도+거리가 부드럽게 변함 (기본) │
  ├─────────────┼───────────────────────────────────────────┤
  │ chaos       │ 여러 주파수 합성, 불규칙하지만 연속적     │                                                     
  ├─────────────┼───────────────────────────────────────────┤                                                     
  │ figure8     │ 8자 패턴, 각도+거리 동시 변화             │                                                     
  ├─────────────┼───────────────────────────────────────────┤                                                     
  │ sweep       │ 단순 좌우 왔다갔다                        │
  └─────────────┴───────────────────────────────────────────┘       

  터미널 3 — 비주얼라이저:
  ros2 run care_robot_control visualizer
