# 자캡디 실차 주행

코드, 모델, 메시지, 펌웨어가 전부 이 폴더에 있다. 컨테이너(`jacapdi-humble`)에는 이 폴더만 같은 경로로 붙인다.

## 주행

```bash
~/jacapdi-docker/car_prep.sh      # 준비: 장치 확인 → 컨테이너 → 파이프라인 → 보드 모니터/송신기 → 화면. 이 창에서 Ctrl+C = 전부 끄기
~/jacapdi-docker/car_go.sh [초]   # 출발 (다른 창). Ctrl+C = 정지. LOG=0 이면 사진 기록 끔, RAW=1 이면 원본도
LANE_AT=8:1,20:2 ~/jacapdi-docker/car_go.sh 60   # 출발 8초 뒤 1차선, 20초 뒤 2차선 자동 변경 (아래 LANE_AT)
~/jacapdi-docker/car.sh stop      # 비상 정지.  car.sh stop all = 파이프라인까지 끄고 요약
~/jacapdi-docker/car.sh all [초]  # 준비 → 엔터 → 출발
~/jacapdi-docker/car.sh lap [초]  # 준비 → 차선 인식 확인 → 5초 카운트다운 → 출발
~/jacapdi-docker/car.sh flash     # 펌웨어 업로드 (필요할 때만)
~/jacapdi-docker/car.sh shell     # 컨테이너 셸 (ROS 환경 켜진 상태)
~/jacapdi-docker/car.sh lane 1|2  # 달릴 차선 (주행 중에도. lane_change_sec 동안 S자로 옮김)
~/jacapdi-docker/car.sh algo <이름>  # 주행 변형 프로필 (목록 = python3 nodes/variant.py --list). 주행 중 전환은 pwm·Stanley 분모가 계단으로 바뀌므로 정지 상태 권장 (경고 출력)
python3 ~/jacapdi-docker/tune_gui.py   # 슬라이더 (주행 중에도 바뀜)
```
화면 끄기: `VIEW=0 car_prep.sh`. YOLO 확신도 기준: `CONF=0.3` (car.sh 맨 위 설정). 출발 차선, 변형: `LANE=1 ALGO=v2 car_prep.sh` (기본 2차선, base).

### 변형 선택 `car.sh algo`

- 쓰는 곳 3곳, 결과는 전부 `tune/algo` 파일 한 줄: `car.sh algo <이름>` (주행 중), `ALGO=<이름> car_prep.sh` (출발 전), tune_gui 변형 버튼
- 경로 노드, 제어기가 다음 프레임에 읽음 (mtime 기준). 두 노드 어긋남 최대 1프레임
- 이름 검증 한 곳: `python3 nodes/variant.py --check <이름>` (exit 0/1). 셸, GUI, sim 전부 이것을 호출. `car.sh algo bogus` = exit 1
- 파일 없음 = `base`. 모르는 이름 = `base` + 로그 1회
- 운전자 UI 목록 = `--list` (base, v1, v2). 벤치 전용 v2a, v2p 는 `--list-all`. v2a 는 벤치 뒤 승격 대상, v2p 는 실험 전용(비권장)
- 주행, 벤치 시작 전 `cat tune/algo` 확인. `v2` 를 남겨 두고 시작 금지 (pipeline.sh, sim 6_repo.sh 미지정 = base)

### 자동 차선 변경 `LANE_AT`

- 형식 `LANE_AT="<초>:<차선>[,<초>:<차선>...]"`. 기준 = car_go.sh 출발 시각. 예 `LANE_AT=8:1,20:2 car_go.sh 60`
- 실차는 위치를 모름 → 시간 기준. 가제보는 `sim/run/7_lane_scenario.sh` 가 호장 s 기준
- 항목 구문 오류 = 경고 후 그 항목만 무시
- 기록 `RUN/lane_events.txt` (`+8s lane 1`). OUT 없어도 RUN 에 남김. 키 1/2 수동 변경과 같이 씀
- 변경 시간 `lane_change_sec` (tune.json·tune_default.json·코드 기본 2.0 = HEAD 값). 10/31 가제보: 2.0·3.0 완주, 4.0 은 왕복 back 11.5 지점에서 두 번째 S자가 우하단 코너와 겹쳐 정지 (수식 노트 4). 실차는 3.0 부터 권장. `LANE_AT` 지정 시 car_go.sh 가 출발 전 2.0~3.5 범위를 확인하고 밖이면 경고 (중단 아님)

## 데이터 흐름

```
webcam_pub ─/image_raw─▶ yolov8_node ─/detections─▶ lane_path_node ─/lane_trajectory─▶ tunable_controller ─/topic_control_signal─▶ serial_sender_node ─▶ 아두이노
                                                           └─/lane_detections─▶ path_viewer            ▲ tune/tune.json, tune/algo, tune/variants.json, tune/lane, tune/go_until(출발 스위치)
```

## 파일

| 위치 | 하는 일 |
|---|---|
| `car_prep.sh`, `car_go.sh` | 호스트: 준비, 출발 (LANE_AT 예약) |
| `car.sh` | 호스트: 공통 설정, 함수(컨테이너 시작, 정지 명령, 송신기, set_file) + stop/all/lap/flash/shell/lane/algo |
| `pipeline.sh` | 컨테이너 안: 웹캠/YOLO/경로/제어기/기록기 시작. `source pipeline.sh env` = ROS 환경만. run_info.txt 에 `variant= lane= path_md5= tune_md5= variants_md5=`. `config/vehicle.yaml` 없으면 시작 전 중단 |
| `nodes/lane_path_node.py` | 선 마스크 → 차로 중앙 경로 곡선. base 동작 + 플래그 가드 (`SolidCross` 실선 넘음 기억·질의, `LaneState` 관측용 상태, `PathResult`) |
| `nodes/tunable_controller.py` | 경로 → 조향/PWM. Stanley + I 항 + 내륜차 보상 + 출발 스위치. 플래그: 속도 스케줄러, 원호 보정, Pure Pursuit |
| `nodes/variant.py` | 변형 프로필 → 플래그 집합 + `Params` 기본값 (단일 출처, ROS 없음). CLI `--list/--list-all/--check/--show` |
| `nodes/vehicle.py`, `config/vehicle.yaml` | 차량 기하 (wheelbase 0.56, cam_to_rear_axle 0.79, ipm_x_near 0.84 → path_origin 1.63, pwm→속도 표 `measured: false` = 시뮬 눈금). 제어기가 읽음 (경로 노드는 IPM 좌표만 쓰므로 안 읽음). `max_steer_deg` 는 실측 ① 기록용 — 제어기 base 는 HEAD 대로 tune.json 값을 씀 |
| `tune/tune.json` | 운전자 키 (tune_gui 슬라이더). 변형 상수는 여기 없음 |
| `tune/variants.json` | 프로필 추가 + `Params` override 만 (git 추적, 저장소 파일은 비어 있음 = 코드 기본값). 기본값 복제 금지(테스트). 유효값은 `variant.py --show` |
| `tune/algo`, `tune/lane` | 런타임 선택 파일 (git 밖). 변형 이름 / 달릴 차선 1,2. 없으면 base / 2 |
| `tests/` | 골든(base, v1, v2) + 단위 테스트 + 셸 정적 검사. 아래 tests 실행법 |
| `nodes/yolov8_node.py` | 사진 → 선 검출 (YOLO 분할, GPU FP16) |
| `nodes/serial_sender_node.py` | 명령 → 아두이노 시리얼 |
| `nodes/ipm.py`, `config/ipm_c920.yaml` | 사진 픽셀 ↔ 바닥 미터 변환 (경로 노드, frame_logger, path_viewer 공용) |
| `nodes/run_monitor.py`, `summarize_run.py`, `frame_logger.py`, `board_monitor.py`, `rate_probe.py`, `path_viewer.py`, `webcam_pub.py` | 기록, 확인. summarize_run.py 는 옛 run(`algo=md5`)과 새 run(`path_md5=`) 둘 다 읽음 |
| `tune/` | 슬라이더 값, 기본값, 프리셋 |
| `models/best.pt` | YOLO 모델 |
| `ros/interfaces_pkg` | 메시지 정의 (컨테이너를 처음 띄울 때 `ros/install` 로 빌드) |
| `arduino/driving_team_cal_1016r` | 지금 보드에 올라간 펌웨어 (.ino 와 같은 내용으로 컴파일한 .hex) |
| `ab_runs/` | 주행 기록 (git 밖) |
| `archive/` | 예전 기록, 백업 (git 밖). 10/30 정리 전 코드는 git 태그 `pre-refactor-20261030` |

## base (HEAD 동작)

- 정의: 프로필 `base` = 플래그 공집합 = 중간평가 2차선 코드 (커밋 `a41706e`)
- 보존 규칙: base 경로에 새 연산, 상수, 순서 변경 금지. 플래그 공집합이면 변형 함수 호출 없음 (`SolidCross` 생성만, 메서드 호출 없음)
- 증명 = 코드 읽기 아님. `tests/golden/base_*.json` (HEAD 모듈로 생성) 과 `==` 비교. isclose 금지. `variants.json` 에 arc_comp 가 있어도 base 는 무시

경로 노드 한 프레임 (2차선 기준, 1차선은 좌우 반전)

| 단계 | 규칙 | 상수 |
|---|---|---|
| 입력 | 사진 지연 0.3 s 초과 또는 마스크 없음 → 실패 | MAX_IMAGE_AGE 0.3 |
| 마스크 | 확신도 0.30 미만 버림. 진행 방향과 60° 넘게 누운 실선 버림 (횡단보도, 정지선) | MIN_SCORE 0.30, TRANSVERSE 60° |
| 경계 | 아래→위 2줄씩. 실선 안쪽 끝 = 오른쪽, 점선 안쪽 끝 = 왼쪽. 점선 빈칸 0.6 m 이하 이어 붙임 | DASH_MAX_GAP 0.6 |
| 선 적합 | 선마다 2차식 (길이 0.3 m 미만 직선). 점 6 미만, 오차 8 cm 초과 → 그 선 버림 | LINE_MAX_ERR 0.08 |
| 중심 | 선 ± 0.425·√(1+y'²). 두 선 폭 허용 0.25 m 안이면 가운데, 아니면 실선 기준 | HALF 0.425, WIDTH_TOL 0.25 |
| 탐색 | 경계 없음 → 중심 오른쪽 0.15 m 경로 (search_right). 2 s 초과 → 실패 | SEARCH_OFFSET 0.15, SEARCH_MAX 2.0 |
| 경로 적합 | 중심점 8 이상. 가까운 0.45 m 우선, 부족하면 0.7 m. 오차 15 cm 초과 → 실패 | MIN_KEEP 8, NEAR 0.45, EXT 0.7 |
| cross_hold | 횡단보도/정지선을 0.5 s 안에 봤는데 search → 믿은 경로 최대 1.5 s | CROSS_RECENT 0.5, CROSS_HOLD_MAX 1.5 |
| outlier_hold | 점이 전부 x 0.4 m 밖 → 믿은 경로. x 0/0.4/0.8 에서 10 cm 넘게 튐 → 0.5 s 유지 뒤 수용 | FAR_ONLY 0.4, JUMP 0.10, OUTLIER_HOLD 0.5 |
| 실패 | 직전 경로 1.0 s 유지 → valid=false (제어기 정지) | HOLD_LAST 1.0 |

제어기 (0.05 s 주기)

- δ = heading_gain·atan(y'(x0)) + atan(stanley_gain·y(x0)/(vehicle_speed_mps + softening_speed_mps)) + curvature_gain·κ(x0) + i_gain·∫y(x0)dt
- 내륜차 보상: c 에 −0.5·L²·κ0 (L 0.56). I 항: |κ| < i_curv_max 이고 포화 아닐 때만 누적, ±i_max_deg
- 조향 = −100·δ/max_steer_deg. PWM 고정 (`pwm`). 경로 lane_timeout_sec 초과 → 정지. 경로 상실 0.5 s → I 항 0
- 운전자 키 (tune.json, HEAD 와 같음): stanley_gain, heading_gain, vehicle_speed_mps, curvature_gain, softening_speed_mps, front_axle_offset_m, max_steer_deg, i_gain, i_max_deg, i_curv_max, lane_timeout_sec, pwm, lane_change_sec
- 시계: `self.now` (기본 `time.time`, 파라미터 `use_ros_clock`). base 기본 동작 불변. 테스트는 주입

## 변형 프로필과 플래그

프로필 (`nodes/variant.py` PROFILES. 표 = `python3 nodes/variant.py --show <이름>` 과 같아야 함)

| 프로필 | 플래그 | 용도 |
|---|---|---|
| `base` | (공집합) | 2차선 중간평가 코드. 회귀 기준 |
| `v1` | one_side_clip, on_line_hold, step_accept, solid_cross_recovery, lane_settle | 1차선 경로 가드 (GUARDS) |
| `v2` | GUARDS + corner_slowdown, speed_schedule | 가드 + 속도 스케줄러. 실차 1차 시험 기본 |
| `v2a` | v2 + arc_comp (기본 limiter=conf) | 평가점 원호 보정 (조향 κ). 벤치 뒤 승격 대상 |
| `v2p` | v2 + controller_pp | Pure Pursuit 제어기. 실험 전용. 1차선 0/3, 2차선 1/3 (조향 권한 부족). 비권장 |

운전자 UI 노출 = `DRIVER_PROFILES` = base, v1, v2. v2a 는 벤치 뒤 승격 대상. `change_slope` 플래그는 존재하지만 어느 프로필에도 없음 (기본 OFF, 효과 미측정, 가정 속도 의존). `v3`(inner_bias) 삭제.

플래그 (목적 / 속한 프로필 / 주요 상수 = `Params` 그룹.키, 기본값)

| 플래그 | 목적 | 프로필 | 주요 상수 |
|---|---|---|---|
| `one_side_clip` | (a) 선 하나로 만든 경로의 \|c\| 상한 | v1 이상 | guard.one_side_max_c_m 0.45 |
| `on_line_hold` | (b) 점선이 x=0 ±near 안(밟고 있음)이면 믿은 경로 유지 | v1 이상 | guard.on_line_half_m 0.25, on_line_hold_sec 0.3, on_line_min_ref_c 0.05 |
| `step_accept` | (c) outlier 수용 시 x 0/0.4/0.8 변화량을 단계씩만 | v1 이상 | guard.accept_step_m 0.3 |
| `short_fit_guard` | (d) 선 하나 ext 적합이 짧거나 기울기 과대면 폐기 | 없음 (기본 OFF, 벤치 뒤 결정) | guard.ext_min_span_m 0.0, ext_max_b 9.0 |
| `solid_cross_recovery` | (e)(f)(g) 실선 넘음 복구: search_back(방향 스냅샷), 후보 창 확장, 연속 확인 뒤 즉시 수용 | v1 이상 | cross.search_back_m 0.3, search_max_sec 1.0, hold_last_sec 0.5, cand_max_cross_m 0.6, c_max_cross_m 0.7, near_m 0.25, max_age_sec 1.5, confirm_frames 2, min_points 15, enter_m −0.2, crossed_m 0.2, overlap_m 0.3 |
| `lane_settle` | 차선 변경 상태기계 IDLE / CHANGING / SETTLING. ramp 끝에 규칙 전환 뒤 오프셋 전 경로 \|c\| < settle_c_m 가 settle_sec 지속(또는 max_sec)이면 IDLE. 변경 중 새 요청은 되돌리기만 즉시, 나머지 보류 | v1 이상 | lane.settle_c_m 0.15, settle_sec 0.5, max_sec 8.0 |
| `change_slope` | 변경 중 S자 오프셋 기울기 d(off)/ds 를 경로 b 에 더함 (ds = assumed_speed·dt) | 없음 (기본 OFF, 효과 미측정) | lane.assumed_speed_mps 0.85 |
| `corner_slowdown` | 속도 스케줄러: \|조향 %\| 히스테리시스 → 목표 pwm 직선 tune pwm / 코너 min(corner.pwm, tune pwm) → 저역통과 (20 Hz 255↔190 왕복 제거, 코너가 직선보다 빨라지지 않음, 출발 스위치 OFF·경로 상실 0.5 s 에 리셋) | v2 이상 | corner.steer_on_pct 60 (진입), steer_off_pct 45 (해제), pwm 190, tau_sec 0.3 |
| `speed_schedule` | 같은 스케줄러 pwm 을 Stanley 분모에 (v_eff = v255·pwm/255, v255 = vehicle.yaml). corner_slowdown 필요(REQUIRES). **vehicle.yaml 이 measured:false 인 동안 분모 눈금이 base(0.5)와 달라 stanley_gain 이 ~1/1.9 로 희석됨** (아래 프로토콜) | v2 이상 | corner.* 공유 |
| `arc_comp` | 평가점 원호 보정. shift = −0.5·(1.63+x0)²·κ_f, 상한 ±shift_max. κ_f = kappa_gain·tan(직전 조향)/L 저역통과. num_points ≤ min_points 면 0. limiter 기본 conf = num_points 신뢰도 가중 (≥ conf_points 전량, min_points+1 에서 conf_min_weight, 사이 선형. 두 선 ≈29점 전량, 선 하나 8~18점 0.5~). 다른 limiter: none, cancel, geo | v2a | arc.kappa_tau_sec 0.4, shift_max_m 0.25, min_points 6, kappa_gain 0.6, limiter conf, conf_points 24, conf_min_weight 0.5 |
| `controller_pp` | Pure Pursuit 분기. κ = 2y_t/D², δ = atan(0.56κ) + I 항. arc_comp 와 배타. 실험 전용 | v2p | pp.lookahead_x_m 0.1, i_gain 0.04, gain 1.0, heading_gain 1.5 |

- 상수 출처: `Params` dataclass (코드) 가 유일한 기본값. `tune/variants.json` = `{"profiles": {이름: [플래그]}, "params": {"cross": {...}, ...}}` 는 **추가·override 만** (저장소 파일은 `{"profiles": {}, "params": {}}`). 기본값과 같은 항목을 적으면 테스트 실패
- 로더: 알 수 없는 키 = 오류 로그 + 무시. 파일 부재·빈 파일·비 UTF-8 = 코드 기본값(노드가 죽지 않음). 배타 `arc_comp`/`controller_pp`, 의존 `speed_schedule → corner_slowdown`, 범위 `CONSTRAINTS`(예 steer_off < steer_on, 0 ≤ pwm ≤ 255) 위반 = 로그 + 제거/그룹 기본값. 제약 `cross.max_age_sec ≥ cross.search_max_sec` 를 `max()` 로 보장 + 경고
- 벤치 재현: 모든 run 에 variant, tune_md5, variants_md5, git_diff_md5 기록
- 삭제됨 (세션 기록에만): v3 inner_bias, arc 모드 1(경로 κ 소스), arc 부호 게이트, tune.json 변형 18키
- 상태 enum `LaneState` (경로 노드): NORMAL / STRADDLE / CROSSED / LOST / FAILED. `SolidCross.state` 가 관측에서 파생하는 **관측용** 값(테스트·진단). 노드 분기는 `SolidCross` 질의 메서드(entered/cand_max/c_limit/search_dir/accept_immediately). 전이표는 클래스 docstring
- `status.json` 키: HEAD 10키 time, steer, pwm, valid, c, b, shift, delta_deg, i_deg, go 그대로 + 추가 kappa_cmd, cte_raw, variant, v_target, arc_shift, v_eff_mps (삭제 없음, 추가만. v_target 은 출발 스위치 OFF 면 null)
- `frames=` 로그 줄: HEAD 접두(`frames= … lane=`) 그대로 + `algo=` + 플래그 통계 `VARIANT_STATS_KEYS` 순서 + `last=` (car_prep.sh 는 `frames=` grep, summarize_run.py 는 줄 전체 복사). 삭제된 v3 의 `inner_biased` 키 없음

## tests 실행법

총 90개 (골든 base/v1/v2, variant, solid_cross, flags, controller, lane_change) + 셸 정적 검사 56.

```bash
cd ~/jacapdi-docker && python3 -m pytest -q tests                # 호스트 (90 passed). tests/conftest.py 가 rclpy 스텁
~/jacapdi-docker/car.sh shell python3 -m pytest tests          # 컨테이너 안 (python3-pytest 는 Dockerfile 에 추가됨 → 이미지 재빌드 필요. 시스템 python numpy 1.21.5 라 저장 골든과 환경이 달라 git a41706e 모듈로 기준을 즉석 생성해 비교함)
cd ~/jacapdi-docker && python3 -m pytest tests/test_base_identity.py   # 병합 게이트 (W0). 매 변경마다
bash ~/jacapdi-docker/tests/test_shell_static.sh                # 셸 정적 검사
python3 tests/make_golden.py --module <py> --out <json>         # 골든 재생성. HEAD 모듈 = git show a41706e:nodes/lane_path_node.py
```

| 파일 | 검사 | 기준 |
|---|---|---|
| `test_base_identity.py` | base 경로 노드 ≥1000 프레임, 30 s 시퀀스(주입 시계), 제어기 격자 | HEAD 골든과 `==` diff 0 (U1~U3), v1/v2 골든 (U4). 골든은 numpy 1.26.4·cv2 4.11 로 생성(meta); 다른 환경이면 같은 인터프리터로 HEAD 모듈을 돌려 기준을 만든다 |
| `test_variant_identity.py` | v1/v2 골든 (LANE_FLAGS 는 골든 생성 시점에 없던 플래그라 뺀다) | U4 |
| `test_lane_change.py` | lane_settle 상태기계 전이, 되돌리기, 보류, change_slope 기울기, base 는 순수 시간 전환 | U10 |
| `test_variant.py` | 부재→base, 모름→base+로그 1, 프로필 포함 관계, variants.json 키 ⊆ dataclass + 기본값 복제 없음, DRIVER_PROFILES ⊆ PROFILES, max_age ≥ search_max, REQUIRES/CONSTRAINTS, 비 UTF-8 바이트, 고정 변형, 미래 mtime | U5 |
| `test_solid_cross.py` | 전이 (i)~(iv), confirm_frames 2, 1프레임 오검출 → ref 불변, 방향 스냅샷 | U6 |
| `test_flags_independent.py` | 단독 플래그에서 해당 stats 키만 증가 | U7 |
| `test_controller.py` | 직선 shift 0, num_points 게이트, kappa_f None, PP 원호 κ 오차 <1e-6, pwm 변화율 ≤65/0.3 s, 코너 pwm ≤ tune pwm (150/190/255), 출발 스위치 OFF 중 스케줄러 리셋, status.json 키 = HEAD 10 + 6 | U8 |
| `test_shell_static.sh` | 셸/GUI/sim 프로필 목록 == `variant.py --list`, run_info 키, LANE_AT 구문 오류 무시 (56개) | U9 |
| `synth.py` | 합성 마스크 (직선, R 1.5~5 m, dy, dyaw, 점선 빈칸, 실선 넘음, 주차선, 횡단보도). sim 의존 없음 | 골든 입력 |

골든 생성 조건: base = HEAD 모듈. v1 = 리팩터 전 작업 트리, `confirm_frames 1` 고정. v2 = `corner_slowdown` 만 (arc 제외).

## 수식 노트

기호: L = 축간 0.56 m, d = 평가점 거리 = 1.63 + x0 (x0 = front_axle_offset_m 0.1 → 1.73 m), R = 곡선 반경, κ = 1/R, v ≈ 0.9 m/s (벤치 랩 39.8 m / 44 s).

1. Stanley 평가점 평형 Δ = d²/(2R)
   - 경로 x=0 은 뒷축 앞 1.63 m (카메라 0.79 + IPM X_near 0.84). 차가 차로 중앙에 정확히 있어도 원호 경로는 평가점에서 d²/(2R) 만큼 안쪽에 보임
   - Stanley 는 cte → 0 으로 수렴 → 차가 안쪽으로 Δ 치우침. R 2 m: 0.75 m, R 3 m: 0.50 m (근사, 정확값 R − √(R²−d²))
   - 1차선 벤치 e 0.33 의 원인. 내륜차 항 0.5·L²·κ (R 2 m 에서 0.08 m) 로는 부족
   - arc_comp: shift = −0.5·d²·κ_f, 상한 0.4 m. κ_f = κ_cmd 저역통과 0.4 s, κ_cmd = tan δ/L (직선에서 0, 노이즈 작음). num_points 게이트
   - 실패 기제 (limiter none, 1차선 0/8): 경로 노드 one_side_clip 0.45 가 R ≤ 2.5 코너의 참 오프셋 0.6~0.75 를 검열 + 시뮬 κ_act/κ_cmd ≈ 0.6 → 보정 과대. limiter conf 로 선 하나(8~18점) 프레임 가중 0.5~ → 1차선 3/3
   - 실차: kappa_gain 을 요레이트/속도 대 tan δ/L 실측으로 맞출 것 (시뮬 추정 0.5~0.65)
2. Pure Pursuit 등가
   - 뒷축에서 목표점 (x_t+1.63, y_t) 까지 현 D² = (1.63+x_t)² + y_t². 뒷축을 지나고 헤딩에 접하는 원호 위 점이면 y_t = D²/(2R) 가 정확히 성립 → κ = 2y_t/D² = 1/R
   - 원호에서 편향 0, a(2차 계수) 불필요, Δ 보정 불필요. arc_comp 의 Δ 뺄셈과 원호에서 같은 답. δ = atan(L·κ)
   - 민감도: y_t 10 cm 오차 → κ 8.6 % (현 Stanley 37.8 %)
3. 관측 한계 R_min ≈ 1.61 m
   - 시야 창: 앞 X_near 0.84 + 0.7 ≈ 1.5 m, 옆 ±0.7 m. 선이 창 옆으로 나가는 반경 R ≈ x²/(2·0.7) = 1.5²/1.4 ≈ 1.61 m
   - 상단 S 구간 R 1.37~1.43 < 1.61 → "관측, 조향 한계 구간". 실차 1차 시험 제외, 카메라 재보정 항목
   - 조향 한계 κ_max = tan(25°)/L = 0.83 → R 1.20 m. 여유 ≤15 %
4. lane_change 물리 한계 κ_ref vs κ_max
   - 옆 이동 0.85 m, smoothstep s²(3−2s) → 최대 가로 가속 ÿ = 6·0.85/T². κ_ref ≈ ÿ/v² = 5.1/(T²·v²)
   - T 2.0 s: κ_ref ≈ 1.5 > κ_max 0.83 → 포화 예측. T 3.0: 0.7. T 4.0: 0.39
   - 10/31 가제보 관측은 예측과 다름: 2.0·3.0 완주, 4.0 은 왕복 back 11.5 지점에서 두 번째 S자가 우하단 코너와 겹쳐 정지. 완료 시간 2→1 5.1~6.3 s, 1→2 4.8~5.1 s 로 lane_change_sec 와 무관 (lane_settle 정착이 지배)
   - 기본 2.0 (HEAD). 실차 3.0 부터 권장. 변경 지점은 직선, 코너 2 m 안 금지
5. 속도 스케줄러
   - 루프 이득 ∝ v. 코너 PWM 190 (정지 시 210). 히스테리시스 진입 60 % / 해제 45 % 조향 상당, 저역통과 0.3 s → pwm 변화율 ≤ 65/0.3 s
   - 같은 v_target 을 Stanley 분모에 (speed_schedule). 분모 불일치 제거

## 가제보 벤치 요약 (10/31, 리팩터 스냅샷)

조건: 격리 4슬롯 (`sim/overnight/matrix`), YOLO conf 0.3, 차량 모델 실차 치수 (축간 0.56, 카메라 0.79, 조향 25°). 통과 = 완주 AND 이탈 0. 수치는 `~/claude/자캡디/sim/logs/matrix/TABLE_final.md` (단일 슬롯 `TABLE_final_single.md`) 참조. 아래는 양상만.

| 조건 | 양상 |
|---|---|
| 2차선 base (HEAD 사본 4회 vs 리팩터 4회) | 동일 양상. 완주 4/4, 이탈 4, 침범 16~18 |
| 2차선 v2 | 8/8 완주, 침범 18, 직선 조향 std 14~18 (base 19~66) |
| 2차선 v2a | 7/8 완주, 침범 3~5, e1 0.05 |
| 1차선 v2 | 15/16 완주, 이탈 3~4, 코너 e1 0.28 |
| 1차선 v2a limiter none | 0/8 (수식 노트 1 실패 기제) |
| 1차선 v2a limiter conf | 3/3 완주, 코너 e1 0.05~0.09, 이탈 1~3 (우코너, 횡단보도 구간 s 28~32) |
| 차선 변경 | 2→1 완료 5.1~6.3 s, 1→2 4.8~5.1 s (lane_change_sec 무관) |

- 엄격 기준(이탈 0) 통과는 아직 없음. 남은 이탈 = (i) 1차선 우코너, 횡단보도 구간 (ii) 2차선 좌코너 점선 접촉
- base 비트 동일: 골든 1040프레임 + 30 s 시퀀스 + 제어기 6250 격자, numpy 1.21/1.26 양쪽 미반박. 골든 밖 차이는 수정 또는 문서화 (비UTF-8 크래시 수정, status.json 키 추가 6, frames= 꼬리 추가, 시계 샘플 위치 HEAD 와 같게, 코너 pwm 상한 min(corner.pwm, tune pwm), 정차 중 스케줄러 리셋)
- 사용자 결정 대기: speed_schedule 의 v_eff 눈금 (절대 vehicle.yaml 1.05 vs tune 상대). 현재 v2 는 Stanley 게인이 base 대비 약 1/1.9

## 실차 1차 시험 프로토콜 (요약, 사양 5절)

측정 (코드 변경보다 먼저, 순서대로)

| # | 항목 | 방법 | 쓰는 곳 |
|---|---|---|---|
| ① | 최대 조향각 좌/우, R_min | PWM 120, s+100 / s−100 원 주행 지름 | `vehicle.yaml max_steer_deg`, 시뮬 steer_limit_deg. R_min > 1.37 m → 상단 S 제외 |
| ② | 조향 스텝 응답 (90 % 시간, 오버슈트), 데드밴드, POT 중심 | 정차, s0→s100→s0→s−100 각 2 s, board_monitor ADC/T | arc.kappa_tau_sec 하한, lag 브리지 delay/rate |
| ③ | PWM→속도 | PWM 150/190/220/255, 5 m 직선 통과 시간. 190 출발 가능 여부 | `vehicle.yaml pwm_speed`, corner.pwm, Stanley 분모 |
| ④ | IPM 재확인 | x=0, 1 m 지점 차로 폭 0.847 m | STRADDLE 판정 (x=0 = 뒷축 앞 1.63 m) |
| ⑤ | 윤거, 타이어 폭, 접지 길이, 카메라 fps, 파이프라인 지연 | 줄자, rate_probe | eval_node 상수, 20 fps 가정 |
| ⑥ | 펌웨어 안전 거동 (바퀴 들고) | 경로 상실→정지 0.3 s, USB 분리 300 ms 정지, 1 Hz 반전 10 s 잠금 없음 | 중단 기준 |

단계 (직전 통과 뒤에만)

0. 체크리스트: `cat tune/algo` = base, `tune/lane` = 2, `variant.py --show base` 공집합, run_info 확인, 보조자 비상정지 위치, 컨테이너 안 `pytest tests` 녹색(같은 인터프리터에서 HEAD == 현재), 출발 직전 `status.json` `pwm`/`steer` 확인(정차 중 조향 포화면 카메라·차 위치 먼저)
1. 2차선 base 회귀: ALGO=base, 중간평가 tune.json, 2바퀴. 수용 1:34 ± 5 s, 침범 비악화. 실패 → 이후 중단 (하드웨어/보정)
2. 1차선 v2 (가드 + 감속, arc OFF, pp OFF): tune pwm ≥ corner.pwm 190 (200 이면 코너 감속 폭이 10 뿐이라 255 또는 corner.pwm 을 pwm−40 으로), 상단 S 제외 1바퀴. 수용 정지 0, 실선 완전 통과 0.
   **알고 가기**: speed_schedule 분모가 vehicle.yaml 시뮬 눈금(255 → 1.05 m/s, measured:false) 이라 같은 stanley_gain 이 base 대비 직선 ~0.52배(PWM 200 ~0.65배) 로 희석된다. 2차선에서 v2 1바퀴를 base 와 비교해 침범이 나쁘면 v2 를 `GUARDS + corner_slowdown` 으로 줄이거나(variants.json 프로필) 측정 ③ 뒤 눈금을 다시 정한다(결정 항목: 절대 눈금 vehicle.yaml vs tune vehicle_speed_mps·pwm/pwm_tune 상대 눈금). 변형 전환은 정지 상태에서만
3. 1차선 v2a (limiter conf): arc.shift_max_m 0.2 시작, 1바퀴. v2p 는 제외 (조향 권한 부족)
4. 2→1 변경 ×3: 직선 구간, lane_change_sec 3.0, LANE_AT 예약 (사전 랩타임으로 초 산출, lane_events.txt 확인). 수용 완료 ≤ 6 s, 바깥 실선 통과 0
5. 1→2 변경 ×3

중단 기준: 바깥 실선 완전 통과 1회, valid 정지 2회, ±50 뒤집힘 1 Hz 이상 3 s, 펌웨어 FAULT 1회, 코너 정지 (토크 부족)

기록: run_info.txt (variant, 해시), line_path.log `frames=`, status.json 시계열 (steer, pwm, cte_raw, arc_shift, kappa_cmd, v_target), board_monitor ADC/T/S/P 0.2 s, frames.csv 10 Hz, lane_events.txt, 외부 영상 1대

금지 (1차 시험): arc 부호 게이트, inner_bias, lane_change_sec 2.0~3.5 밖 (실차 3.0 부터), 코너 2 m 안 차선 변경, 상단 S 구간, 측정 ①~③ 없이 arc/pp, v2p. 시뮬 통과 = "시뮬 조건부 통과" 까지만
