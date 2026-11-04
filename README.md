# 자율주행캡스톤디자인 공유 저장소

## 주행

```bash
~/jacapdi-docker/car_prep.sh      # 준비: 장치 확인 → 컨테이너 → 파이프라인 → 보드 모니터/송신기 → 화면. 이 창에서 Ctrl+C = 전부 끄기
~/jacapdi-docker/car_go.sh [초]   # 출발 (다른 창). Ctrl+C = 정지. LOG=0 이면 사진 기록 끔, RAW=1 이면 원본도
~/jacapdi-docker/car.sh stop      # 비상 정지.  car.sh stop all = 파이프라인까지 끄고 요약
~/jacapdi-docker/car.sh all [초]  # 준비 → 엔터 → 출발
~/jacapdi-docker/car.sh lap [초]  # 준비 → 차선 인식 확인 → 5초 카운트다운 → 출발
~/jacapdi-docker/car.sh flash     # 펌웨어 업로드 (필요할 때만)
~/jacapdi-docker/car.sh shell     # 컨테이너 셸 (ROS 환경 켜진 상태)
python3 ~/jacapdi-docker/tune_gui.py   # 슬라이더 (주행 중에도 바뀜)
```
화면 끄기: `VIEW=0 car_prep.sh`. YOLO 확신도 기준: `CONF=0.3` (car.sh 맨 위 설정).

## 데이터 흐름

```
webcam_pub ─/image_raw─▶ yolov8_node ─/detections─▶ lane_path_node ─/lane_trajectory─▶ tunable_controller ─/topic_control_signal─▶ serial_sender_node ─▶ 아두이노
                                                           └─/lane_detections─▶ path_viewer            ▲ tune/tune.json, tune/go_until(출발 스위치)
```

## 파일

| 위치 | 하는 일 |
|---|---|
| `car_prep.sh`, `car_go.sh` | 호스트: 준비, 출발 |
| `car.sh` | 호스트: 공통 설정·함수(컨테이너 시작, 정지 명령, 송신기) + stop/all/lap/flash/shell |
| `pipeline.sh` | 컨테이너 안: 웹캠/YOLO/경로/제어기/기록기 시작. `source pipeline.sh env` = ROS 환경만 |
| `nodes/lane_path_node.py` | 선 마스크 → 차로 중앙 경로 곡선 (횡단보도·튐 대응, 차선 변경, 서 있는 장애물 차 피해 차선 변경 포함) |
| `nodes/tunable_controller.py` | 경로 → 조향/PWM (Stanley + I 항 + 내륜차 보상, 출발 스위치). 설정 12개 = `tune/tune.json` |
| `nodes/yolov8_node.py` | 사진 → 선 검출 (YOLO 분할, GPU FP16) |
| `nodes/serial_sender_node.py` | 명령 → 아두이노 시리얼 |
| `nodes/ipm.py`, `config/ipm_c920.yaml` | 사진 픽셀 ↔ 바닥 미터 변환 (경로 노드, frame_logger, path_viewer 공용) |
| `nodes/calib_ipm.py` | 카메라를 다시 달았을 때 `config/ipm_c920.yaml` 재보정 (렌즈 높이 실측 + 직선 2차선에서 pitch 풀기) |
| `nodes/run_monitor.py`, `summarize_run.py`, `frame_logger.py`, `board_monitor.py`, `rate_probe.py`, `path_viewer.py`, `webcam_pub.py` | 기록·확인 |
| `tune/` | 슬라이더 값, 기본값, 프리셋 |
| `models/best.pt` | YOLO 모델 |
| `ros/interfaces_pkg` | 메시지 정의 (컨테이너를 처음 띄울 때 `ros/install` 로 빌드) |
| `arduino/driving_team_cal_1016r` | 지금 보드에 올라간 펌웨어 (.ino 와 같은 내용으로 컴파일한 .hex) |
| `ab_runs/` | 주행 기록 (git 밖) |
| `archive/` | 예전 기록·백업 (git 밖). 10/30 정리 전 코드는 git 태그 `pre-refactor-20261030` |
