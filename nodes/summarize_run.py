# -*- coding: utf-8 -*-
"""주행 요약: timeline.csv + line_path.log → 화면에 출력하고 <기록폴더>/../results.csv 에 한 줄 추가.
pipeline.sh 가 끝날 때 실행. 따로: python3 summarize_run.py ~/jacapdi-docker/ab_runs/<기록폴더>

run_info.txt 두 형식을 읽는다 (read_run_info):
  옛 run: algo=lane_path_node md5=<md5>              → path_md5=<md5>, variant/lane 은 "?"
  새 run: variant= lane= path_md5= tune_md5= variants_md5=  (pipeline.sh)
results.csv 가 이미 있으면 그 머리행의 열만 쓴다 (열 추가로 기존 표가 어긋나지 않게)."""
import csv
import sys
from pathlib import Path

INFO_KEYS = ("variant", "lane", "path_md5", "tune_md5", "variants_md5")


def read_run_info(path):
    """run_info.txt → dict. 옛 키 algo=(경로 노드 md5) 는 path_md5 로 옮기고, 없는 키는 "?"."""
    info = dict(l.split("=", 1) for l in Path(path).read_text().splitlines() if "=" in l)
    old = info.get("algo", "")                 # "lane_path_node md5=abc..." (옛 pipeline.sh)
    if "path_md5" not in info and "md5=" in old:
        info["path_md5"] = old.split("md5=", 1)[1].split()[0]
    for k in INFO_KEYS:
        info.setdefault(k, "?")
    return info


def main():
    L = Path(sys.argv[1])
    info = read_run_info(L / "run_info.txt")
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
        "tag": L.name, "algo": info["variant"], "conf": info.get("conf", "?"),
        "duration_s": "%.1f" % (float(rows[-1]["t"]) if rows else 0.0), "traj_valid_pct": "%.1f" % (100.0 * sum(valid) / n),
        "longest_invalid_s": "%.1f" % (longest * 0.1),
        "speed0_pct": "%.1f" % (100.0 * sum(r["speed"] in ("", "0") for r in rows) / n),
        "mean_abs_steer": "%.2f" % (sum(steer) / max(1, len(steer))),
        "no_solid_pct": "%.1f" % (100.0 * sum(r["n_solid"] == "0" for r in rows) / n),
        "yolo_hz": "%.1f" % (sum(hz) / len(hz)) if hz else "",
        "search_timeout_logs": log.count("search_timeout"),
        "line_path_stats": stats[-1].split("]: ", 1)[-1] if stats else "",
    }
    summ.update((k, info[k]) for k in INFO_KEYS)
    for k, v in summ.items():
        print("%-20s %s" % (k, v))
    res = L.parent / "results.csv"
    fields = list(summ)
    if res.is_file():                          # 기존 표의 열 그대로 (새 열은 새 표에서만)
        with open(res, newline="") as f:
            head = next(csv.reader(f), None)
        fields = head or fields
    new = not res.is_file()
    with open(res, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerow(summ)
    print("추가됨:", res)


if __name__ == "__main__":
    main()
