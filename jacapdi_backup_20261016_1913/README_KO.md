# 10/16 19:13 백업 (10/10 14:36 주행 설정으로 되돌리기 직전)

10/10 best drive 로 되돌리기 전에 지금 상태를 그대로 복사해 둔 것. 다시 쓰려면 원래 자리(`~/jacapdi-docker/`, `tune/tune.json`)에 덮어쓰면 됩니다.

| 되돌린 파일 | 이 백업(10/16) | 지금(10/10 으로 되돌림) |
|---|---|---|
| `tunable_controller.py`, `tune_gui.py` | Stanley I 항(i_gain) 추가, 모터 보정 범위 ±80 | 10/10 `code_used/` 판 |
| `car_common.sh` | 펌웨어 1010P (조향 PD) | 펌웨어 1010F |
| `tune/tune.json` | motor_balance 74, 최대 조향 20°, 인코스 -0.15, lane_guard 0.09/2, i_gain 0.02 | `preset_best_20261010_1436.json` |

그 밖의 파일(고스트·바닥 보정 코드 등)은 되돌리지 않았고, 참고로 같이 복사만 해 둠.
