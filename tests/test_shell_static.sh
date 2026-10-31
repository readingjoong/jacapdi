#!/usr/bin/env bash
# 셸·기록 정적 검사 (도커·웹캠·ROS 없이 호스트에서). 사용: bash tests/test_shell_static.sh   (STRICT=1: 다른 작업자 파일의 하드코딩도 실패로)
#   1. bash -n: car.sh car_prep.sh car_go.sh pipeline.sh
#   2. 프로필 목록 하드코딩 없음 (base|v1 같은 열거는 nodes/variant.py 에만). tune_gui.py·sim/run/6_repo.sh 는 경고(STRICT=1 이면 실패)
#   3. car.sh algo bogus → exit 1, tune/algo 안 바뀜. nodes/variant.py 가 있으면 algo base → exit 0 + tune/algo == base. 인자 없음 → 사용법 + exit 1
#   4. pipeline.sh DRY=1 드라이런 → run_info.txt 에 variant= lane= path_md5= tune_md5= variants_md5= + 기존 키 tag= conf= webcam=, 사본 파일. config/vehicle.yaml 없으면 exit 1
#   5. car_go.sh lane_sched: LANE_AT 구문 오류 항목 무시, 올바른 항목은 set_lane + lane_events.txt (OUT 없이 RUN 에). 출발 전 lane_change_sec ≥ 4.0 검사 존재
#   6. summarize_run.py: 옛 run(algo=lane_path_node md5=) 과 새 run 둘 다 읽어 results.csv 한 줄씩
# 저장소 파일은 건드리지 않는다: 임시 폴더에 셸 파일을 복사해 거기서 돈다 (car.sh 의 D = 임시 폴더).
set -u
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${SIM:-$HOME/claude/자캡디/sim}"
STRICT="${STRICT:-0}"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
pass=0; fail=0; warnc=0
ok()   { pass=$((pass + 1)); printf '  ok    %s\n' "$*"; }
bad()  { fail=$((fail + 1)); printf '  FAIL  %s\n' "$*"; }
wrn()  { warnc=$((warnc + 1)); printf '  warn  %s\n' "$*"; }
check() { if "$@" >/dev/null 2>&1; then ok "${*:1:3}"; else bad "${*:1:3}"; fi; }   # 조건 명령으로 판정

echo "[1] bash -n"
for f in car.sh car_prep.sh car_go.sh pipeline.sh; do
  bash -n "$R/$f" && ok "bash -n $f" || bad "bash -n $f"
done

echo "[2] 프로필 목록 하드코딩 없음"
PAT='base\|v1|v1\|v2|v2\|v3|\("base", *"v1"|\[.base., *.v1.'   # 셸 case 열거(base|v1 …)·파이썬 튜플/리스트 열거
for f in car.sh car_prep.sh car_go.sh pipeline.sh nodes/summarize_run.py; do
  if grep -nE "$PAT" "$R/$f" >/dev/null; then bad "하드코딩 $f: $(grep -nE "$PAT" "$R/$f" | head -1)"; else ok "하드코딩 없음 $f"; fi
done
for f in "$R/tune_gui.py" "$SIM/run/6_repo.sh"; do
  [ -f "$f" ] || continue
  if grep -nE "$PAT" "$f" >/dev/null; then
    if [ "$STRICT" = 1 ]; then bad "하드코딩 $f (다른 작업자 파일)"; else wrn "하드코딩 $f (다른 작업자 파일, STRICT=1 이면 실패): $(grep -nE "$PAT" "$f" | head -1 | cut -c1-80)"; fi
  else ok "하드코딩 없음 $(basename "$f")"; fi
done
grep -q "variant.py.*--check" "$R/car.sh" && ok "car.sh 가 variant.py --check 사용" || bad "car.sh 에 variant.py --check 없음"
grep -q "variant.py.*--check" "$R/pipeline.sh" && ok "pipeline.sh 가 variant.py --check 사용" || bad "pipeline.sh 에 variant.py --check 없음"
grep -qE "sed -n '[0-9]+,[0-9]+p'" "$R/car.sh" && bad "car.sh 사용법이 sed 줄 번호 고정" || ok "car.sh 사용법 줄 번호 고정 없음"

# 임시 저장소 사본: 셸 파일 + nodes/{variant,lane_path_node,summarize_run}.py + tune/
mkdir -p "$T/repo/nodes" "$T/repo/tune" "$T/repo/config"
cp "$R"/car.sh "$R"/car_prep.sh "$R"/car_go.sh "$R"/pipeline.sh "$T/repo/"
cp "$R"/nodes/lane_path_node.py "$R"/nodes/summarize_run.py "$T/repo/nodes/"
[ -f "$R/nodes/variant.py" ] && cp "$R/nodes/variant.py" "$T/repo/nodes/"
cp "$R/tune/tune.json" "$T/repo/tune/"
cp "$R/config/vehicle.yaml" "$T/repo/config/"
[ -f "$R/tune/variants.json" ] && cp "$R/tune/variants.json" "$T/repo/tune/"
echo v2 > "$T/repo/tune/algo"; echo 1 > "$T/repo/tune/lane"   # 이전 값이 남은 상태를 흉내
HAVE_VARIANT=0; [ -f "$T/repo/nodes/variant.py" ] && HAVE_VARIANT=1

echo "[3] car.sh algo"
bash "$T/repo/car.sh" algo bogus >/dev/null 2>&1; rc=$?
[ $rc = 1 ] && ok "car.sh algo bogus → exit 1" || bad "car.sh algo bogus → exit $rc (1 이어야)"
[ "$(cat "$T/repo/tune/algo")" = v2 ] && ok "bogus 뒤 tune/algo 안 바뀜" || bad "bogus 가 tune/algo 를 바꿈: $(cat "$T/repo/tune/algo")"
bash "$T/repo/car.sh" algo "" >/dev/null 2>&1; rc=$?
[ $rc = 1 ] && ok "car.sh algo '' → exit 1" || bad "car.sh algo '' → exit $rc"
if [ $HAVE_VARIANT = 1 ]; then
  bash "$T/repo/car.sh" algo v3 >/dev/null 2>&1; rc=$?           # 차선 범위 프로필도 이름 검증 한 곳(variant.py --check)으로 통과
  [ $rc = 0 ] && [ "$(cat "$T/repo/tune/algo")" = v3 ] && ok "car.sh algo v3 (차선 범위 프로필) → exit 0, tune/algo=v3" || bad "car.sh algo v3 → exit $rc, tune/algo=$(cat "$T/repo/tune/algo")"
  bash "$T/repo/car.sh" algo base >/dev/null 2>&1; rc=$?
  [ $rc = 0 ] && [ "$(cat "$T/repo/tune/algo")" = base ] && ok "car.sh algo base → exit 0, tune/algo=base" || bad "car.sh algo base → exit $rc, tune/algo=$(cat "$T/repo/tune/algo")"
  n=$(python3 "$R/nodes/variant.py" --list | wc -l)
  [ "$n" -ge 1 ] && ok "variant.py --list 가 $n 개 프로필" || bad "variant.py --list 비어 있음"
else
  wrn "nodes/variant.py 없음 (W1 진행 중) → algo base 성공 경로는 건너뜀"
fi
out=$(bash "$T/repo/car.sh" 2>&1); rc=$?
[ $rc = 1 ] && echo "$out" | grep -q "car.sh algo" && echo "$out" | grep -q "car.sh stop" && ok "car.sh 인자 없음 → 사용법 + exit 1" || bad "car.sh 사용법 출력/exit 이상 (rc=$rc)"
echo "$out" | grep -q '^#' && bad "사용법에 # 가 남음" || ok "사용법에 주석 기호 없음"

echo "[4] pipeline.sh DRY=1"
RUNDIR="$T/run_new"
out=$(cd "$T" && DRY=1 RUN="$RUNDIR" bash "$T/repo/pipeline.sh" 2>&1); rc=$?
[ $rc = 0 ] && ok "pipeline.sh DRY=1 → exit 0" || { bad "pipeline.sh DRY=1 → exit $rc"; echo "$out" | sed 's/^/        /'; }
for k in tag variant lane path_md5 tune_md5 variants_md5 conf webcam; do
  grep -q "^$k=" "$RUNDIR/run_info.txt" 2>/dev/null && ok "run_info.txt $k=" || bad "run_info.txt 에 $k= 없음"
done
grep -q "^algo=" "$RUNDIR/run_info.txt" 2>/dev/null && bad "run_info.txt 에 옛 algo= 가 남음" || ok "run_info.txt 옛 algo= 없음 (path_md5 로 개명)"
[ "$(grep ^variant= "$RUNDIR/run_info.txt" | cut -d= -f2)" = base ] && ok "ALGO 미지정 → variant=base" || bad "ALGO 미지정인데 variant=$(grep ^variant= "$RUNDIR/run_info.txt")"
[ "$(cat "$T/repo/tune/algo")" = base ] && ok "ALGO 미지정 → tune/algo 를 base 로 덮어씀" || bad "tune/algo 가 $(cat "$T/repo/tune/algo")"
[ "$(grep ^lane= "$RUNDIR/run_info.txt" | cut -d= -f2)" = 2 ] && ok "LANE 미지정 → lane=2" || bad "lane=$(grep ^lane= "$RUNDIR/run_info.txt")"
md5=$(md5sum "$T/repo/nodes/lane_path_node.py" | cut -d' ' -f1)
grep -q "^path_md5=$md5$" "$RUNDIR/run_info.txt" && ok "path_md5 == md5(lane_path_node.py)" || bad "path_md5 불일치"
for f in tune_at_start.json algo_at_start lane_at_start; do [ -f "$RUNDIR/$f" ] && ok "사본 $f" || bad "사본 $f 없음"; done
[ -f "$T/repo/tune/variants.json" ] && { [ -f "$RUNDIR/variants_at_start.json" ] && ok "사본 variants_at_start.json" || bad "variants_at_start.json 없음"; }
# ALGO 지정 (variant.py 있을 때만 검증 가능), 잘못된 LANE/ALGO 는 exit 1
(cd "$T" && DRY=1 RUN="$T/run_x" LANE=3 bash "$T/repo/pipeline.sh" >/dev/null 2>&1); rc=$?
[ $rc = 1 ] && ok "pipeline.sh LANE=3 → exit 1" || bad "pipeline.sh LANE=3 → exit $rc"
(cd "$T" && DRY=1 RUN="$T/run_y" ALGO=bogus bash "$T/repo/pipeline.sh" >/dev/null 2>&1); rc=$?
[ $rc = 1 ] && ok "pipeline.sh ALGO=bogus → exit 1" || bad "pipeline.sh ALGO=bogus → exit $rc"
mv "$T/repo/config/vehicle.yaml" "$T/repo/config/vehicle.yaml.off"
(cd "$T" && DRY=1 RUN="$T/run_z" bash "$T/repo/pipeline.sh" >/dev/null 2>&1); rc=$?
[ $rc = 1 ] && ok "pipeline.sh config/vehicle.yaml 없음 → exit 1 (제어기 임포트 실패 전에 멈춤)" || bad "pipeline.sh vehicle.yaml 없음 → exit $rc"
mv "$T/repo/config/vehicle.yaml.off" "$T/repo/config/vehicle.yaml"
if [ $HAVE_VARIANT = 1 ]; then
  first=$(python3 "$R/nodes/variant.py" --list | head -1)
  (cd "$T" && DRY=1 RUN="$T/run_v" LANE=1 ALGO="$first" bash "$T/repo/pipeline.sh" >/dev/null 2>&1); rc=$?
  [ $rc = 0 ] && grep -q "^variant=$first$" "$T/run_v/run_info.txt" && grep -q "^lane=1$" "$T/run_v/run_info.txt" && [ "$(cat "$T/repo/tune/algo")" = "$first" ] \
    && ok "pipeline.sh ALGO=$first LANE=1 → 기록·tune/algo 일치" || bad "pipeline.sh ALGO=$first → exit $rc"
fi

echo "[5] car_go.sh lane_sched (LANE_AT)"
EVDIR="$T/run_ev"; mkdir -p "$EVDIR"
out=$(cd "$T" && bash -c '
  source "'"$T"'/repo/car_go.sh" >/dev/null 2>&1 || exit 9
  RUN="'"$EVDIR"'"; OUT=""
  LANE_AT="abc:1,0:9,0,:2,0.0:2,0:1x,0:1" lane_sched
  echo "rc=$?"' 2>&1); rc=$?
echo "$out" | grep -q "rc=0" && ok "lane_sched 가 구문 오류 항목에서 죽지 않음" || { bad "lane_sched 종료 코드 (source rc=$rc)"; echo "$out" | sed 's/^/        /'; }
n=$(echo "$out" | grep -c "LANE_AT 항목 무시")
[ "$n" = 5 ] && ok "구문 오류 5개 모두 경고 뒤 무시" || bad "무시 경고 $n 개 (5 이어야): $(echo "$out" | tr '\n' '|' | cut -c1-200)"
[ "$(cat "$T/repo/tune/lane")" = 1 ] && ok "올바른 항목 순서대로 적용 → tune/lane=1" || bad "tune/lane=$(cat "$T/repo/tune/lane")"
[ -f "$EVDIR/lane_events.txt" ] && [ "$(wc -l < "$EVDIR/lane_events.txt")" = 2 ] && ok "OUT 없이 RUN/lane_events.txt 2줄" || bad "RUN/lane_events.txt: $(cat "$EVDIR/lane_events.txt" 2>/dev/null | wc -l)줄"
grep -q "lane 2" "$EVDIR/lane_events.txt" 2>/dev/null && grep -q "lane 1" "$EVDIR/lane_events.txt" && ok "lane_events.txt 내용 (lane 2, lane 1)" || bad "lane_events.txt 내용"
grep -q "차선, 변형" "$R/car_go.sh" && ok "car_go.sh 출발 메시지에 차선·변형" || bad "car_go.sh 출발 메시지에 차선·변형 없음"
if [ $HAVE_VARIANT = 1 ]; then   # LANE_AT(set_lane) 이 쓴 tune/lane 을 Variant 가 그대로 읽어 v3 의 차선별 플래그를 고름 (셸에 차선 규칙 복제 없음)
  echo v3 > "$T/repo/tune/algo"
  fl=$(cd "$T/repo" && python3 -c "import sys; sys.path.insert(0, 'nodes'); import variant as V; v = V.Variant('tune'); print(v.lane, int(v.flags == V.PROFILES['v3'].lane1))" 2>&1)
  [ "$fl" = "1 1" ] && ok "LANE_AT 뒤 tune/lane=1 → v3 가 1차선 플래그로 풀림" || bad "v3 차선 풀림: $fl (1 1 이어야)"
  echo base > "$T/repo/tune/algo"
fi
awk '/lane_change_sec/ && !seen {seen=NR} /go_until"   # 출발!/ {go=NR} END {exit !(seen && go && seen < go)}' "$R/car_go.sh" \
  && ok "car_go.sh: LANE_AT 의 lane_change_sec ≥ 4.0 검사가 출발(go_until) 전에 있음" || bad "car_go.sh: lane_change_sec 검사 없음 또는 출발 뒤"

echo "[6] summarize_run.py 옛/새 run_info"
mkdir -p "$T/ab/old_run" "$T/ab/new_run"
printf 'tag=old_run\nalgo=lane_path_node md5=0123456789abcdef0123456789abcdef\nconf=0.3\nwebcam=/dev/video0\n' > "$T/ab/old_run/run_info.txt"
printf 't,traj_valid,steer,speed,n_solid,det_hz\n0.0,1,10,120,1,20\n0.1,0,,0,0,20\n0.2,1,-5,120,2,20\n' > "$T/ab/old_run/timeline.csv"
printf '[INFO] [x.y] [lane_path_node]: frames=30 valid=28 outlier=1 search_timeout=0\n' > "$T/ab/old_run/line_path.log"
cp "$RUNDIR/run_info.txt" "$T/ab/new_run/run_info.txt" 2>/dev/null || printf 'tag=new_run\nvariant=base\nlane=2\npath_md5=aaaa\ntune_md5=bbbb\nvariants_md5=none\nconf=0.3\nwebcam=none\n' > "$T/ab/new_run/run_info.txt"
cp "$T/ab/old_run/timeline.csv" "$T/ab/new_run/"; cp "$T/ab/old_run/line_path.log" "$T/ab/new_run/"
o1=$(python3 "$R/nodes/summarize_run.py" "$T/ab/old_run" 2>&1); rc1=$?
o2=$(python3 "$R/nodes/summarize_run.py" "$T/ab/new_run" 2>&1); rc2=$?
[ $rc1 = 0 ] && echo "$o1" | grep -q "path_md5 *0123456789abcdef" && ok "옛 run: algo=lane_path_node md5= → path_md5" || { bad "옛 run 요약 (rc=$rc1)"; echo "$o1" | sed 's/^/        /'; }
[ $rc2 = 0 ] && echo "$o2" | grep -q "^variant *base" && echo "$o2" | grep -q "^lane *2" && ok "새 run: variant/lane 읽음" || { bad "새 run 요약 (rc=$rc2)"; echo "$o2" | sed 's/^/        /'; }
echo "$o1" | grep -q "frames=30" && ok "line_path.log frames= 줄 파싱 유지" || bad "frames= 파싱"
[ "$(wc -l < "$T/ab/results.csv")" = 3 ] && ok "results.csv 머리행 + 2줄" || bad "results.csv $(wc -l < "$T/ab/results.csv")줄"
head -1 "$T/ab/results.csv" | grep -q "variant,lane,path_md5,tune_md5,variants_md5" && ok "results.csv 새 열" || bad "results.csv 머리행: $(head -1 "$T/ab/results.csv")"
# 기존 표(옛 머리행)가 있으면 그 열만 쓴다
printf 'tag,algo,conf\nx,y,z\n' > "$T/ab/results.csv"
python3 "$R/nodes/summarize_run.py" "$T/ab/new_run" >/dev/null 2>&1
[ "$(tail -1 "$T/ab/results.csv" | tr ',' '\n' | wc -l)" = 3 ] && ok "기존 results.csv 열 수 유지 (3)" || bad "기존 results.csv 에 열이 어긋남: $(tail -1 "$T/ab/results.csv")"

echo
echo "통과 $pass, 실패 $fail, 경고 $warnc"
[ $fail = 0 ]
