# tests/ — base 비트 보존 골든 안전망 (사양 REFACTOR_SPEC_1031 2.4, W0)

리팩터가 **base(HEAD a41706e, 중간평가 2차선 코드)** 와 **벤치된 변형(v1 경로 가드, v2 코너 감속)** 의 출력을 바꾸지 않았음을
저장소 안에서 자동으로 증명한다. 비교는 전부 `==` (isclose 없음).

## 실행

```sh
# 컨테이너 안
./car.sh shell python3 -m pytest -q tests
# 호스트(시뮬 env 의 파이썬: rclpy·numpy·cv2)
cd ~/jacapdi-docker && set +u && source ~/claude/자캡디/sim/env.sh && "$JPY" -m pytest -q tests
# pytest 가 없으면 파일 단위로
"$JPY" tests/test_base_identity.py
# 동결 사본으로 "골든·테스트 자체가 맞는지" 확인 (작업 트리가 다른 작업자 때문에 깨졌을 때)
JACAPDI_NODES_DIR=~/claude/자캡디/sim/logs/matrix/repo_baseline_snapshot/nodes "$JPY" -m pytest -q tests/test_base_identity.py tests/test_variant_identity.py
```

rclpy·interfaces_pkg 는 `golden_lib.install_ros_stubs()` 가 항상 스텁으로 바꿔 끼우므로 ROS 가 없는 환경에서도 같은 결과가 나온다
(numpy·cv2·yaml 만 필요). 골든은 numpy 1.26.4 / cv2 4.11.0 / Python 3.10 으로 생성했고 `meta.numpy`·`meta.cv2` 에 적혀 있다.

**골든은 생성 환경에 묶인다.** 다른 numpy/cv2(컨테이너 시스템 python 은 numpy 1.21.5·cv2 4.5.4; pipeline.sh 가 노드를 그 python 으로 띄움)
에서는 polyfit·래스터 결과가 ULP 이상으로 달라 저장 골든과 어긋난다 — 그것은 리팩터 결함이 아니다. `identity_lib.load_golden` 은 환경이 다르면
같은 인터프리터로 기준 모듈(base = `git show a41706e`, v1/v2 = 동결 사본 `meta.module`)을 돌려 기준 데이터를 즉석 생성해 비교한다
(git·사본이 없으면 skip 하고 사유를 출력). 결국 어느 환경에서든 '기준 모듈 == 현재 모듈' 을 같은 인터프리터로 증명한다.
컨테이너 안 실행에는 `python3-pytest` 가 필요하다(Dockerfile 에 추가, 이미지 재빌드).

## 파일

| 파일 | 내용 |
|---|---|
| `synth.py` | 바닥 좌표의 선(직선, R 1.5~5 m 좌/우 호, 직선→호 진입, 호→직선, dy {0,±0.1,0.4,0.6}, dyaw {0,±8,10}, 점선 0.55/0.45 m 주기, 실선 넘은 포즈, 선 밟은 포즈, 횡단보도(정상 클래스/실선 오인), 주차 구획선 오인, 먼 곳만, 선 하나) 을 `Ipm.to_px` 로 640×480 마스크에 그려 YOLO 폴리곤 흉내. **1040 프레임**, 순서 고정 |
| `golden_lib.py` | ROS 스텁, 주입 시계 `CLOCK`, 모듈 로더(`load_module`: 모듈의 `time` 을 가짜 모듈로, `<repo>/tune` 아래 문자열 상수를 임시 폴더로), 프레임/30 s 시퀀스/제어기 격자 실행기, 시퀀스 대본 `sequence_steps` |
| `make_golden.py` | 골든 생성. `--regen-all` 이 HEAD 를 `git show` 로 꺼내 base 골든을, 동결 사본으로 v1/v2 골든을 만든다 |
| `golden/base_path.json` | HEAD 모듈, tune/algo 없음, HEAD tune.json. 프레임마다 `[이름, lane, transverse_dropped, [reason, a, b, c, x_min, x_max, fit_err, num_points, band(24점)]]` |
| `golden/base_seq.json` | HEAD 모듈 `on_detections` 30 s(600 프레임, 주입 시계 20 fps). 프레임마다 `[t, last_reason, 발행 traj 필드 [valid,a,b,c,x_min,x_max,err,n], 발행 band 점]` + `stats` 12키 |
| `golden/base_ctrl.json` | HEAD 제어기. tune 두 벌(A = HEAD tune.json, B = 게인·pwm 변경, i_gain 0) × 격자 (a 5 × b 5 × c 5) × num_points {0,5,6,7,40} × prev_steer {−100,−60,0,60,100} = 3125/벌 → `[a,b,c,n,prev, steer, pwm_l, pwm_r, shift(status.json), i_acc]`; `seq` = I 누적·포화 → 경로 상실 0.7 s → 복귀 → 곡선 134틱 |
| `golden/v1_path.json`, `v1_seq.json` | 동결 사본 모듈, algo=v1, 사본 tune.json(벤치가 돌아간 디스크 값: search_max 1.0, hold_last 0.5, ext_min_span 0.0, ext_max_b 9.0) |
| `golden/v2_ctrl.json` | 동결 사본 제어기, algo=v2, 사본 tune.json(arc_comp 0 → corner_slowdown 만) |
| `identity_lib.py` | 골든 비교 공용 (첫 diff 위치를 경로로 출력) |
| `test_base_identity.py` | U1~U3. 현재 모듈을 algo 부재 / `base` 두 조건으로 돌려 base 골든과 ==. tune.json 에 옛 변형 키(arc_comp 2 포함)와 variants.json 에 arc_comp 켜진 프로필을 넣어도 무시됨을 같은 비교로 확인. 변형 stats 는 전부 0 이어야 함 |
| `test_variant_identity.py` | U4. v1 경로(프레임·시퀀스, stats base 12키 + v1 6키) 와 v2 제어기 골든 ==. 골든의 `inner_biased`(삭제된 v3) 는 현재 stats 에 없음을 확인 |
| `test_lane_scope.py` | 차선 범위 프로필(LaneScope, 새 v3 = 1차선 V3_LANE1(= v2f) / 2차선 base). tune/lane·tune/lane_eff 읽기 규칙, 플래그 차선 = 경로 노드 Lane 실제 상태(실제 Lane+Variant 가짜 시계: SETTLING 중 보류된 요청 2→1→2·1→2→1, 정착 성공·타임아웃, scope_hold 는 S자 전체 1차선), variants.json lane1/lane2 형식, CLI --show 차선별 줄, 상태 플래그(corner_slowdown·arc_comp)가 켜지거나 꺼질 때만 제어기 상태 리셋. 골든: v3 2차선 변경 없음 == base(경로 프레임·시퀀스·제어기), 골든 시퀀스 요청 전 프레임 ==, 1차선 == v2f |

## 시퀀스 대본이 지나는 분기 (base_seq 기준)

믿은 경로 확립(0–3 s) → 0.3 m 튐: outlier_hold 0.5 s → outlier_accepted(v1: outlier_step) → 먼 곳만: far_only_hold → ref 1 s 만료 →
횡단보도 + 경계 없음: cross_hold 1.5 s → search_right → search_timeout(2.0 s; v1 1.0 s) → held 1.0 s(v1 0.5 s) → valid=false →
stale(0.5 s) → no_mask → no_line(점수 0.2) → 횡단보도를 실선으로(transverse_dropped) → R3/R2 호 → solid_only(v1 c_clip) → 점선 밟음(v1 on_line_hold) →
차선 변경 2→1(lane 파일, 2 s S 자, publish 의 off 가산) → 1차선 실선 밟음(v1 solid_on_line_accept) → 실선 넘음 + 바깥 점선(v1 (f) 창·|c| 0.7) →
경계 없음(v1 search_back_ref / base search_left) → 타임아웃 → 복귀.

## 골든 재생성 (동작을 바꾸는 결정이 난 뒤에만)

```sh
"$JPY" tests/make_golden.py --regen-all            # base ← git a41706e, v1/v2 ← 동결 사본
"$JPY" tests/make_golden.py --kind path --module nodes/lane_path_node.py --algo v1 --tune tune/tune.json --out /tmp/x.json
```

base 골든은 저장소 HEAD 가 a41706e 인 한 다시 만들 일이 없다. v1/v2 골든은 "리팩터 전 작업 트리" 의 동작이므로, 변형 동작을 바꾸기로
결정한 뒤에만 새 기준으로 교체한다(세션 기록에 이유).

## 리팩터 작업자(W1/W2/W3)에게 — 테스트가 기대하는 계약

- 노드 공개 진입점: `LanePathNode()` (스텁 Node 로 생성 가능해야 함: `create_subscription/publisher/timer`, `get_logger`, `get_clock`, `declare_parameter`),
  `setup(h, w)`, `find_path(md, ms, header, mirror)` (3-튜플 또는 `.traj/.band/.reason` 을 가진 객체), `on_detections(msg)`, `is_transverse(poly)`, `stats` dict, `last_reason`.
- 제어기: `Tunable("")`, `on_traj(msg)`, `tick()`, 속성 `prev_steer`, `i_acc`, `n`(status.json 쓰기 주기 카운터). status.json 의 `shift` 키.
- 발행 토픽 이름 `/lane_trajectory`, `/lane_detections`, `/topic_control_signal` (스텁 퍼블리셔가 토픽으로 모은다).
- 시계: `time.time()` / `time.monotonic()` 을 모듈 전역 `time` 으로 호출(또는 기본값이 `time.time` 인 `self.now`). 테스트는 모듈의 `time` 이름을 바꿔 주입한다.
- tune 경로: `<repo>/tune` 아래 파일 경로를 **모듈(또는 nodes/ 의 보조 모듈 ipm/variant/vehicle/tunefile) 의 문자열 상수**로 두면 테스트가 임시 폴더로 바꾼다.
  다른 방식(함수 안에서 경로 조립 등)으로 바꾸면 `golden_lib.set_tune_dir` 도 같이 고쳐야 한다.
- v1 골든은 즉시 수용이 1프레임인 동작이다. 리팩터 뒤 `confirm_frames` 기본 2 가 되면 `test_variant_identity.V1_VARIANTS` 가 쓰는 variants.json
  (`{"params": {"cross": {"confirm_frames": 1}}}`) 로 1 을 주고 비교한다. 로더 형식이 다르면 그 상수 하나만 고친다.
