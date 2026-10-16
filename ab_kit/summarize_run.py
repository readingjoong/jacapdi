# -*- coding: utf-8 -*-
"""start_ab.sh 종료 때 자동 실행: timeline.csv + 로그로 요약을 출력하고 ~/jacapdi-docker/ab_runs/results.csv 에 한 줄 추가.
따로 실행도 된다:  python3 summarize_run.py ~/jacapdi-docker/ab_runs/<기록폴더>"""
import csv
import os
import re
import sys
from pathlib import Path


def main():
    L = Path(sys.argv[1])
    info = dict(l.split("=", 1) for l in (L / "run_info.txt").read_text().splitlines() if "=" in l and not l.startswith(" "))
    rows = list(csv.DictReader(open(L / "timeline.csv"))) if (L / "timeline.csv").is_file() else []
    dur = float(rows[-1]["t"]) if rows else 0.0
    valid = [int(r["traj_valid"]) for r in rows]
    vr = 100.0 * sum(valid) / max(1, len(valid))
    run = best = 0
    for v in valid:
        run = run + 1 if not v else 0; best = max(best, run)
    moving = [r for r in rows if r["speed"] not in ("", "0")]
    stop_pct = 100.0 * (1 - len(moving) / max(1, len(rows)))
    steer = [abs(int(r["steer"])) for r in rows if r["steer"] != ""]
    hz = [float(r["det_hz"]) for r in rows[20:]] if len(rows) > 30 else []
    no_solid = 100.0 * sum(1 for r in rows if r["n_solid"] == "0") / max(1, len(rows))
    stats = ""
    lp = L / "line_path.log"
    if lp.is_file():
        s = [l for l in lp.read_text(errors="replace").splitlines() if "frames=" in l]
        stats = s[-1].split("]: ", 1)[-1] if s else ""
    timeouts = len(re.findall(r"search_timeout", lp.read_text(errors="replace"))) if lp.is_file() else 0
    summ = {
        "tag": L.name, "algo": info.get("algo", "?").split()[0], "conf": info.get("conf", "?"),
        "duration_s": "%.1f" % dur, "traj_valid_pct": "%.1f" % vr, "longest_invalid_s": "%.1f" % (best * 0.1),
        "speed0_pct": "%.1f" % stop_pct, "mean_abs_steer": "%.2f" % (sum(steer) / max(1, len(steer))),
        "no_solid_pct": "%.1f" % no_solid, "yolo_hz": "%.1f" % (sum(hz) / len(hz)) if hz else "",
        "search_timeout_logs": timeouts, "line_path_stats": stats,
    }
    for k, v in summ.items():
        print("%-20s %s" % (k, v))
    res = Path(os.path.expanduser("~/jacapdi-docker/ab_runs/results.csv"))
    new = not res.is_file()
    with open(res, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ));
        if new:
            w.writeheader()
        w.writerow(summ)
    print("추가됨:", res)


if __name__ == "__main__":
    main()
