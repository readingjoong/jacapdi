# 주행 당시 슬라이더 값 (2026-10-10 14:36 주행)

| 슬라이더 (KO / EN) | 키 | 값 | 범위 | 설명 |
|---|---|---|---|---|
| Stanley 게인 / Stanley gain | `stanley_gain` | **1.15** | 0.0 ~ 3.0 | 차로 중심에서 벗어난 거리에 대한 반응. 크면 세게 복귀(흔들림 증가) |
| 방향 게인 / Heading gain | `heading_gain` | **0.2** | 0.0 ~ 2.0 | 경로 방향과 차 방향 차이에 대한 반응. 크면 흔들림 감쇠·커브 진입 빠름 |
| 속도 가정 / Assumed speed (m/s) | `vehicle_speed_mps` | **0.5** | 0.2 ~ 3.0 | Stanley 식의 속도. 크면 거리 오차 반응이 약해짐 |
| 곡률 보정 / Curvature gain | `curvature_gain` | **0.55** | 0.0 ~ 2.0 | 경로가 휘는 만큼 미리 조향 (커브 대응) |
| 저속 완화 / Softening speed | `softening_speed_mps` | **0.1** | 0.05 ~ 1.0 | 저속에서 조향이 튀는 것 완화 |
| 평가 지점 / Look-ahead offset (m) | `front_axle_offset_m` | **0.1** | -0.8 ~ 0.8 | 경로를 평가하는 앞뒤 위치 (0 = 카메라 보정 근거리 끝) |
| 최대 조향각 / Max steer angle (deg) | `max_steer_deg` | **25.0** | 10 ~ 40 | 이 각도 = 끝까지(100%). 실측 바퀴 최대 25°. 작을수록 더 많이 꺾음 / Smaller = steers more |
| 최대 조향 단계 / Max steer step | `max_steering_step` | **7** | 1 ~ 7 | 조향 상한 (예전 단계 단위, 7 = 100%) |
| 조향 트림 / Steer trim (step) | `steer_trim` | **0.0** | -3 ~ 3 | 직진이 한쪽으로 흐를 때 보정 (+ 오른쪽). 0.1 단계 = 1.4% |
| 경로 좌우 이동 / Path offset (m) | `path_offset_m` | **0.04** | -0.3 ~ 0.3 | + 이면 경로를 왼쪽으로 (오른쪽 실선 침범 시) |
| 인코스 / Inside bias (m) | `inside_bias_m` | **0.05** | -0.2 ~ 0.2 | + 커브 안쪽으로, - 바깥쪽으로. 커브에서 안쪽 점선 밟으면 - 로 / + inside, - outside |
| 인코스-오른쪽 커브 / Inside bias right (m) | `inside_bias_right_m` | **0.0** | -0.2 ~ 0.2 | 0 = 위와 같게. - 면 오른쪽 커브를 늦게·크게 돎 / - = later, wider right turns |
| 급발진 방지 / Steer rate limit (step/0.05s) | `steer_rate` | **0.0** | 0 ~ 7 | 0.05초에 몇 단계까지 바꿀지 (소수 가능). 1 = 부드럽게, 0 = 제한 없음 |
| 좌우 모터 보정 / Motor balance (PWM) | `motor_balance` | **2** | -30 ~ 30 | 직선에서 오른쪽으로 흐르면 + (왼쪽 모터 줄임), 왼쪽으로 흐르면 - |
| 멈춤 탈출 / Stall boost (PWM) | `stall_boost_pwm` | **255** | 0 ~ 255 | 1초 동안 차가 안 움직이면 0.5초 이 PWM 으로. 0 = 끔 / 0 = off |
| 선 넘기 방지-오른쪽 / Lane guard right (m) | `lane_guard_m` | **0.0** | 0.0 ~ 0.3 | 실선 쪽으로 이만큼 벗어나면 오른쪽으로 더 못 꺾음. 0 = 끔 |
| 선 넘기 방지-되돌리기 / Lane guard push (step) | `lane_guard_push` | **0** | 0 ~ 4 | 기준 넘으면 반대쪽으로 최소 이 단계. 0 = 막기만 / 0 = block only |
| 선 넘기 방지-왼쪽 / Lane guard left (m) | `lane_guard_left_m` | **0.0** | 0.0 ~ 0.3 | 점선 쪽 기준. 0 = 오른쪽과 같게. 인코스 쓰면 오른쪽보다 크게 (0.18) |
| 경로 끊김 허용 / Path timeout (s) | `lane_timeout_sec` | **0.6** | 0.1 ~ 1.0 | 이 시간 넘게 경로 없으면 정지 |

슬라이더에 없는 값: `steering_sign`=-1.0

## 조향 단계별 뒷바퀴 PWM / Rear PWM per steer step

| 단계 0 | 단계 1 | 단계 2 | 단계 3 | 단계 4 | 단계 5 | 단계 6 | 단계 7 |
|---|---|---|---|---|---|---|---|
| 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 |
