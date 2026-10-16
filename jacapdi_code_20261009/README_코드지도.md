# 자캡디 실차 전체 코드 (2026-10-09 기준)

차를 움직이는 데 쓰는 코드 전부를 한 곳에 모았습니다. 파이썬·아두이노·실행 스크립트·설정을 모두 포함합니다.
**여기 파일은 복사본입니다.** 고친 뒤에는 아래 "원래 위치"에 덮어써야 실제 주행에 반영됩니다.
모든 코드를 한 파일로 이어 붙인 `ALL_CODE.md` 도 있습니다(읽기용).

## 전체 흐름

```
웹캠 C920 ─▶ webcam_pub.py ─/image_raw─▶ yolov8_node.py (팀, best.pt)
                                              │ /detections (실선·점선 등 모양 + 확신도)
                                              ▼
                              F_outlier_hold.py (경로 알고리즘 F = E + 튐 무시)
                                              │ /lane_trajectory (경로 곡선 y = a·x² + b·x + c)
                                              ▼
   tune_gui.py (슬라이더) ─tune.json─▶ tunable_controller.py (Stanley + 슬라이더 기능)
                                              │ /topic_control_signal (조향 단계 -7~7, 뒷바퀴 PWM)
                                              ▼
                              serial_sender_node.py (팀) ─USB "s3l255r255"─▶ 아두이노 1009b
                                                                              ├ 조향 모터 + 가변저항 (A2)
                                                                              └ 뒷바퀴 모터 2개
```

## 무엇을 고치려면 어디를?

| 하고 싶은 것 | 파일 | 위치 (함수/줄 근처) |
|---|---|---|
| 실선·점선으로 경로 만드는 방법 | `python/1_path/E_curve_cross.py` | `find_path()`, `on_detections()` |
| 경로가 튈 때 무시하는 규칙, 횡단보도 | `python/1_path/F_outlier_hold.py` | `find_path()` |
| 실선/점선 확신도 기준, 가로선 각도 | `E_curve_cross.py` | `__init__` 의 `solid_min_score`, `dashed_min_score`, `transverse_deg` |
| 조향 계산 (Stanley), 인코스, 선 넘기 방지, 급발진 방지, 멈춤 탈출 | `python/2_control/tunable_controller.py` | `tick()` 의 (1)~(6) 단계 |
| Stanley 공식 자체 | `python/2_control/team_lane_keeping_controller.py` | `compute_delta()` |
| 슬라이더 항목 추가/범위 | `python/3_slider_gui/tune_gui.py` | 맨 위 `CTRL` 목록 |
| 조향 가운데·끝 값, 조향 모터 힘, 고장 판정 | `arduino/driving_team_cal_1009b/*.ino` | 맨 위 상수들 (`POT_CENTER`, `STEERING_PWM` …) |
| 카메라 → 바닥 거리 변환 (IPM) | `config/ipm_c920.yaml` | 카메라 위치/각도 바뀌면 다시 보정 (`calib_pitch.py`) |
| 준비/주행 순서, 기본 알고리즘·YOLO 기준 | `scripts/car_common.sh`, `car_prep.sh`, `car_go.sh` | `ALGO`, `CONF`, `FW_HEX` |

## 원래 위치 (고친 뒤 여기에 덮어쓰기)

| 이 묶음 | 노트북의 원래 위치 |
|---|---|
| `python/1_path/*.py` (lane_from_lines 제외) | `~/jacapdi-docker/ab_kit/algo/` |
| `python/1_path/lane_from_lines.py` | `~/jacapdi-live-car-2026-09-24/labeling-tools/` |
| `python/2_control/tunable_controller.py`, `python/3_slider_gui/tune_gui.py`, `python/4_tools/*.py` (run_monitor 제외) | `~/jacapdi-docker/` |
| `python/4_tools/run_monitor.py`, `scripts/start_ab.sh` | `~/jacapdi-docker/ab_kit/` |
| `python/2_control/team_lane_keeping_controller.py` | `~/team_ws/src/decision_making_pkg/decision_making_pkg/lib/lane_keeping_controller.py` (팀 원본, 고치면 빌드 필요) |
| `python/team_original/*.py` | `~/team_ws/src/...` (팀 원본, 참고용) |
| `scripts/*.sh` | `~/jacapdi-docker/` |
| `config/ipm_c920.yaml`, `drive_controller_tunable.yaml` | `~/jacapdi-docker/` |
| `config/tune_*.json`, `config/presets/` | `~/jacapdi-docker/tune/` (`tune_current.json` = 지금의 `tune.json`) |
| `arduino/driving_team_cal_1009b/` | `~/jacapdi-live-car-2026-09-24/arduino/` |

## 언제 무엇을 다시 해야 하나

| 고친 것 | 반영 방법 |
|---|---|
| 슬라이더 값 | 바로 반영 (0.05초 안에) |
| `tunable_controller.py`, 경로 알고리즘 `*.py`, `*.yaml` | `car_prep.sh` 다시 실행 |
| 아두이노 `.ino` | Arduino IDE 로 컴파일·업로드, 또는 새 hex 를 만들고 `car_common.sh` 의 `FW_HEX`/`FW_NAME` 을 바꾼 뒤 `car_flash.sh` |

## 펌웨어 두 가지
- `1009b`: **지금 차에 올라가 있는 것.** 가운데 ADC 462 (실측), 조향 모터 힘 180
- `1009c`: 1009b + 조향 모터 힘 255 (더 빨리 꺾음). 아직 실차에서 안 써 봄
