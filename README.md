# Care Robot Controller — ROS2 Jazzy

## 패키지 구조

```
care_robot_ws/
└── src/
    ├── care_robot_controller/
    │   ├── care_robot_controller/
    │   │   ├── person_follower_node.py    # 메인 제어 노드
    │   │   ├── neck_controller_node.py    # XC330 Dynamixel 제어
    │   │   └── robot_simulator_node.py    # 유니사이클 시뮬레이터 + 비전 mock
    │   ├── scripts/
    │   │   └── visualizer.py              # matplotlib 실시간 시각화
    │   ├── launch/
    │   │   └── sim_launch.py
    │   ├── setup.py
    │   └── package.xml
    └── care_robot_msgs/
        └── msg/
            └── PersonDetection.msg        # 비전 모듈 ↔ 제어 인터페이스
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

## 핵심 파라미터 (URDF 기반)

| 항목 | 값 | 출처 |
|------|-----|------|
| 바퀴 반경 | 0.0325 m | URDF `wheel_radius` |
| 휠베이스 | 0.2115 m | URDF `left_wheel_y * 2` |
| 목 yaw 위치 | x=0.1244, z=0.2358 m | URDF `neck_x, neck_z` |
| 카메라 FOV (수평) | ~120° (2.094 rad) | ArduCam IMX708 spec |
| 목표 거리 | 1.0 m ± 0.1 m | 요구사항 |
| 비전 업데이트 | 15 Hz | 일반적 depth cam |
| 제어 루프 | 50 Hz | |

---

## 제어 구조

### FSM (Finite State Machine)
```
         detection          timeout (0.5s)
  IDLE ──────────▶ FOLLOWING ──────────▶ SEARCHING
   ▲                   ▲                     │
   │                   │   detection         │ timeout (5s)
   └───────────────────┘◀────────────────────┘
```

### Loop 1: 거리 제어 (P+D)
```
e_r = r - r_d  (r_d = 1.0m)
v   = Kp_v * e_r + Kd_v * ė_r    |  deadband ±0.1m
```

### Loop 2: 헤딩 제어 (P+D)
```
ω = Kp_w * φ + Kd_w * φ̇
φ: 목 yaw 프레임에서 사람 각도
```

### Loop 3: 목 yaw (position)
```
θ_neck = φ  (직접 각도 추적)
|φ| < 5°이면 neck → 0 복귀 (body가 처리)
```

---

## 빌드 & 실행

```bash
# 1. 빌드
cd ~/care_robot_ws
colcon build --symlink-install --packages-select care_robot_msgs care_robot_controller
source install/setup.bash

# 2. 시뮬레이션 실행
ros2 launch care_robot_controller sim_launch.py

# person_mode 변경 (circle / linear / still)
ros2 launch care_robot_controller sim_launch.py person_mode:=linear person_speed:=0.5

# 3. 시각화 (별도 터미널)
source install/setup.bash
python3 src/care_robot_controller/scripts/visualizer.py

# 4. 상태 모니터링
ros2 topic echo /robot_state
ros2 topic hz /cmd_vel
ros2 topic echo /person_detection_raw
```

---

## 실제 하드웨어 전환 체크리스트

### 비전 모듈 연동 (친구 파트)
- [ ] `care_robot_msgs/PersonDetection` 빌드 및 공유
- [ ] 친구 노드가 `/person_detection` (또는 `/person_detection_raw`) 퍼블리시
- [ ] `angle` 기준 확인: **neck_yaw_link 프레임, +CCW(왼쪽)**

### 바퀴 모터 (JGB37-520 + Arduino)
- [ ] Arduino → ROS2 브리지: `/cmd_vel` 구독 → 좌/우 모터 PWM 변환
  ```
  v_L = (v - ω * L/2) / r_w   [rad/s]
  v_R = (v + ω * L/2) / r_w   [rad/s]
  ```
- [ ] 엔코더 피드백 → `/odom` 퍼블리시 (optional for this task)
- [ ] v_max, omega_max 실측 후 재조정

### 목 모터 (XC330-T181)
- [ ] `neck_controller_node.py` 파라미터 수정:
  ```yaml
  simulation_mode: false
  device_name: /dev/ttyUSB0   # USB2Dynamixel 또는 U2D2
  baud_rate: 57600
  dxl_id: 1
  ```
- [ ] `pip install dynamixel-sdk`
- [ ] Dynamixel Wizard로 ID, baud 확인

### 게인 튜닝 순서
1. `Kp_v` 단독 튜닝 (사람 정면, 거리만 변화)
2. `Kp_w` 단독 튜닝 (사람 각도만 변화)
3. `Kd_*` 추가 (overshoot 있을 때만)
4. `motor_tau` 실측 (step response로)

---

## 주의사항

- 비전 각도 기준이 **neck_yaw_link 프레임**인지 **base_link 프레임**인지 친구와 반드시 합의
- XC330은 TTL Half-Duplex → 일반 USB-Serial 어댑터 불가, **U2D2** 또는 **USB2Dynamixel** 필요
- 실내 운용 속도이므로 v_max = 0.4 m/s 적용 (JGB37-520 최대 0.85 m/s 대비 여유)