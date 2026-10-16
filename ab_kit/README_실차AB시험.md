# 실차 경로 알고리즘 A/B 시험 키트 (2026-09-27)

경로 생성 알고리즘 4종(A~D)과 YOLO threshold(conf)를 명령 한 줄로 바꿔 가며 실차에서 비교하는 키트입니다.
구성은 기존 `start_pipeline.sh`와 같습니다(웹캠 → YOLO best.pt → 경로 생성 → Stanley). 모터(시리얼)는 기존처럼 따로 켭니다.
기존 파일은 하나도 바꾸지 않습니다.

| 버전 | 내용 |
|---|---|
| A 원본 | 인계받은 `line_path_node.py` 그대로. 오른쪽 실선이 안 보이면 오른쪽 탐색 2초 뒤 정지 |
| B 점선 대체 | 오른쪽 실선이 안 보여도 점선이 보이면 점선 기준으로 계속 감. 둘 다 없을 때만 탐색. 실선이 보이는 동안은 A와 똑같이 동작 |
| C 커브 보정 | B + 커브에서 차로 중심을 선의 수직 방향으로 계산 (원본은 커브에서 중심이 바깥쪽 선에 붙음) |
| D 안정화 | C + 경로가 프레임 사이에 갑자기 튀면 버리고 부드럽게 이음, 선이 다 사라지면 1초 유지 후 탐색. 반응이 조금 느림 |

가제보 결과(`compare_ABCD.png`, 첫 바퀴): A conf 0.5는 7번 S자 구간에서 정지(실차와 같은 곳), B/C/D는 conf 0.5에서 완주, 앞바퀴 최대 오차 D 4.3 cm < C 6.7 cm < B 14.5 cm.
가제보의 PWM-속도와 조향각은 가정값이라 실차 결과를 대신하지 않습니다.

## 1. 설치 (실차 노트북 호스트에서 한 번)

압축을 푼 폴더에서:

```bash
cp -r ab_kit ~/jacapdi-docker/ab_kit
```

`~/jacapdi-docker`는 도커 컨테이너 안에 같은 경로로 연결돼 있어서 컨테이너에서도 보입니다.

## 2. 점검 (컨테이너 안, 카메라 없이 30초)

```bash
~/jacapdi-docker/run.sh
```

```bash
bash ~/jacapdi-docker/ab_kit/check_ab.sh
```

마지막 줄이 `결과: 모두 정상`이면 됩니다. 직선에서는 넷이 같은 경로를 내고, 실선이 없을 때 A만 `search_right`로 가야 정상입니다.

## 3. 실행

```bash
ALGO=B CONF=0.5 bash ~/jacapdi-docker/ab_kit/start_ab.sh
```

- `ALGO`: A, B, C, D (기본 A) / `CONF`: YOLO threshold (기본 0.5)
- 경로 보기: 다른 탭에서 `~/jacapdi-docker/run.sh` 로 들어가 `python3 ~/jacapdi-docker/path_viewer.py`
- 모터: 기존 방법 그대로 (serial_sender 또는 `timed_drive.sh`)
- 바퀴 띄운 첫 점검은 약한 설정으로: `CTRL_YAML=~/jacapdi-live-car-2026-09-24/config/bench_controller.yaml ALGO=B bash ~/jacapdi-docker/ab_kit/start_ab.sh` (PWM 40, 조향 한 단계)
- 끝낼 때 Ctrl+C: 요약이 나오고 `~/jacapdi-docker/ab_runs/results.csv`에 한 줄이 쌓입니다 (경로 유효율, 가장 긴 경로 끊김, 정지 명령 비율, 실선 미검출 비율, YOLO 속도)
- `RECORD=1`: 결과 토픽(검출/경로/명령)을 rosbag으로 기록 (작음)
- `RECORD=2`: 카메라 영상까지 기록 (1분에 약 1.6 GB). 가져오면 같은 실제 영상으로 A~D를 다시 비교할 수 있습니다

## 4. 시험 순서 (`ab_kit/시험기록표.csv`)

1. 바퀴 띄우고 A, B, C, D를 10초씩: 경로가 뷰어에 나오는지, 조향 방향이 맞는지
2. 지면 주행, 같은 출발점, conf 0.5로 A → B → C → D 한 바퀴씩. 이어서 2회차
3. 시간이 남으면 conf 0.3으로 A, B
4. 매 주행 기록: 결과(완주/정지/개입), 멈춘 구간 번호(`ab_kit/track_sections.png`의 1~8), 선 밟은 횟수, 사람 개입 횟수, 영상 파일 이름, 기록 폴더 이름(start_ab.sh가 처음에 출력하는 tag)

## 5. 주의

- B/C/D는 점선만 보고도 달립니다. 점선을 잘못 잡으면(횡단보도 줄무늬, 주차선) 따라갈 수 있으니 5번(횡단보도) 구간을 잘 보세요.
  원본의 "실선이 없으면 오른쪽 탐색" 규칙을 만든 이유가 따로 있었다면 팀 안에서 먼저 확인하세요.
- conf를 낮추면 흐린 선까지 잡지만 잘못 잡는 것도 늘어납니다. 가제보에서 A conf 0.3은 통과할 때도 있고 멈출 때도 있었습니다.
- 안전 규칙은 기존 그대로: 처음엔 바퀴 부양과 PWM 40, 정지 명령(s0l0r0) 확인 뒤에 지면 주행.

## 6. 되돌리기

키트는 기존 파일을 바꾸지 않습니다. 원래 파이프라인은 그대로 `bash ~/jacapdi-docker/start_pipeline.sh` 입니다.
`ab_kit/algo/A_orig.py`는 원본과 같은 동작입니다(시간을 재는 함수만 노드 시계로 바꿨고, 실차에서는 원래와 같습니다).
