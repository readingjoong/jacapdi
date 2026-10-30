# -*- coding: utf-8 -*-
"""주행 요약: timeline.csv + line_path.log → 화면에 출력하고 <기록폴더>/../results.csv 에 한 줄 추가.
pipeline.sh 가 끝날 때 실행. 따로: python3 summarize_run.py ~/jacapdi-docker/ab_runs/<기록폴더>"""
import csv
import sys
from pathlib import Path


def main():
    L = Path(sys.argv[1])
    info = dict(l.split("=", 1) for l in (L / "run_info.txt").read_text().splitlines() if "=" in l)
    rows = list(csv.DictReader(open(L / "timeline.csv"))) if (L / "timeline.csv").is_file() else []
    valid = [int(r["traj_valid"]) for r in rows]
    run = longest = 0                          # 가장 길게 이어진 경로 없음 (10 Hz 줄 수)
    for v in valid:
        run = 0 if v else run + 1; longest = max(longest, run)
    steer = [abs(int(r["steer"])) for r in rows if r["steer"] != ""]
    hz = [float(r["det_hz"]) for r in rows[20:]] if len(rows) > 30 else []   # 처음 2 초(예열) 제외
    log = (L / "line_path.log").read_text(errors="replace") if (L / "line_path.log").is_file() else ""
    stats = [l for l in log.splitlines() if "frames=" in l]
    n = max(1, len(rows))
    summ = {
        "tag": L.name, "algo": info.get("algo", "?").split()[0], "conf": info.get("conf", "?"),
        "duration_s": "%.1f" % (float(rows[-1]["t"]) if rows else 0.0), "traj_valid_pct": "%.1f" % (100.0 * sum(valid) / n),
        "longest_invalid_s": "%.1f" % (longest * 0.1),
        "speed0_pct": "%.1f" % (100.0 * sum(r["speed"] in ("", "0") for r in rows) / n),
        "mean_abs_steer": "%.2f" % (sum(steer) / max(1, len(steer))),
        "no_solid_pct": "%.1f" % (100.0 * sum(r["n_solid"] == "0" for r in rows) / n),
        "yolo_hz": "%.1f" % (sum(hz) / len(hz)) if hz else "",
        "search_timeout_logs": log.count("search_timeout"),
        "line_path_stats": stats[-1].split("]: ", 1)[-1] if stats else "",
    }
    for k, v in summ.items():
        print("%-20s %s" % (k, v))
    res = L.parent / "results.csv"
    new = not res.is_file()
    with open(res, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ))
        if new:
            w.writeheader()
        w.writerow(summ)
    print("추가됨:", res)


if __name__ == "__main__":
    main()
