# -*- coding: utf-8 -*-
"""흰 선 라벨(solid_line, dashed_line)에서 주행 영역 `lane` 폴리곤을 자동 생성해 YOLOv8 세그 데이터셋에 붙인다.

입력: Roboflow YOLOv8 세그 내보내기 폴더 (data.yaml + train/valid/test/{images,labels})
출력: 같은 구조의 새 폴더. 라벨 파일 끝에 `lane` 클래스 폴리곤 한 줄 추가, data.yaml names에 lane 추가,
      qa/ 폴더에 확인용 오버레이 PNG, report.json에 통계.

원리: 선 폴리곤을 마스크로 그리고, 이미지 아래(차 앞)에서 위로 한 줄씩 훑으며 현재 차로 중심의
      왼쪽에서 가장 가까운 선 조각과 오른쪽에서 가장 가까운 선 조각을 찾는다. 점선 사이 빈 줄은
      위아래 값으로 보간한다. 두 경계 사이를 채운 영역이 lane이다. 한쪽 선만 보이는 이미지는
      lane을 만들지 않고 report에 남긴다(그 이미지는 lane 학습에서 빼는 편이 안전).

사용:
  python lane_from_lines.py <내보내기 폴더> <출력 폴더> [--boundary solid_line dashed_line] [--qa 40]
  python lane_from_lines.py --selftest <출력 폴더>      # 합성 데이터로 동작 확인

정적 검사와 합성 시험만 했다. 실제 라벨 분포에서는 --min-width, --max-gap 조정이 필요할 수 있다.
"""
import argparse
import json
import os
import random
import shutil
import sys

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # data.yaml을 간단 파서로 읽는다
    yaml = None

LANE_NAME = "lane"


# --------------------------------------------------------------------- data.yaml
def read_names(data_yaml):
    text = open(data_yaml, encoding="utf-8").read()
    if yaml is not None:
        d = yaml.safe_load(text)
        names = d.get("names")
        if isinstance(names, dict):
            names = [names[k] for k in sorted(names, key=int)]
        return d, list(names)
    names = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("names:"):
            inner = line[len("names:"):].strip().strip("[]")
            names = [n.strip().strip("'\"") for n in inner.split(",") if n.strip()]
    return {"names": names}, names


def write_names(src_yaml, dst_yaml, d, names):
    d = dict(d)
    d["names"] = names
    d["nc"] = len(names)
    if yaml is not None:
        open(dst_yaml, "w", encoding="utf-8").write(yaml.safe_dump(d, allow_unicode=True, sort_keys=False))
    else:
        lines = [l for l in open(src_yaml, encoding="utf-8").read().splitlines()
                 if not l.strip().startswith(("names:", "nc:"))]
        lines.append("nc: %d" % len(names))
        lines.append("names: [%s]" % ", ".join("'%s'" % n for n in names))
        open(dst_yaml, "w", encoding="utf-8").write("\n".join(lines) + "\n")


# --------------------------------------------------------------------- 라벨 파일
def read_label(path):
    """YOLO 세그 라벨: 줄마다 `cls x1 y1 x2 y2 ...` (정규화). (cls, [(x,y),...]) 목록."""
    out = []
    if not os.path.isfile(path):
        return out
    for line in open(path, encoding="utf-8"):
        t = line.split()
        if len(t) < 7:
            continue
        cls = int(float(t[0]))
        vals = list(map(float, t[1:]))
        pts = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
        out.append((cls, pts))
    return out


def polygon_to_mask(polys, w, h):
    mask = np.zeros((h, w), np.uint8)
    for pts in polys:
        arr = np.array([[int(round(x * w)), int(round(y * h))] for x, y in pts], np.int32)
        if len(arr) >= 3:
            cv2.fillPoly(mask, [arr], 255)
    return mask


# --------------------------------------------------------------------- 핵심
def runs_in_row(row, gap=3):
    """한 행에서 선 픽셀의 연속 구간 [(xl, xr), ...]. gap 픽셀 이하 끊김은 같은 구간."""
    xs = np.flatnonzero(row)
    if xs.size == 0:
        return []
    runs = []
    start = prev = int(xs[0])
    for x in xs[1:]:
        x = int(x)
        if x - prev > gap + 1:
            runs.append((start, prev))
            start = x
        prev = x
    runs.append((start, prev))
    return runs


def scan_lane(mask, min_width, max_width, max_gap_rows, start_x=None):
    """아래에서 위로 훑어 행별 (left_edge, right_edge). 없는 행은 None.
    반환: L[h], R[h] (np.float 배열, nan = 없음), 통계 dict."""
    h, w = mask.shape
    L = np.full(h, np.nan)
    R = np.full(h, np.nan)
    mid = (w - 1) / 2.0 if start_x is None else float(start_x)
    last_both = None
    rows_since_any = 0
    stat = {"rows_both": 0, "rows_left_only": 0, "rows_right_only": 0, "rows_none": 0}
    for y in range(h - 1, -1, -1):
        runs = runs_in_row(mask[y])
        left = [r for r in runs if r[1] < mid]
        right = [r for r in runs if r[0] > mid]
        lx = max(left, key=lambda r: r[1])[1] if left else None
        rx = min(right, key=lambda r: r[0])[0] if right else None
        if lx is not None and rx is not None:
            width = rx - lx
            if width < min_width or width > max_width:
                stat["rows_none"] += 1
                rows_since_any += 1
            else:
                L[y], R[y] = lx, rx
                mid = (lx + rx) / 2.0
                last_both = y
                rows_since_any = 0
                stat["rows_both"] += 1
                continue
        elif lx is not None:
            L[y] = lx
            stat["rows_left_only"] += 1
            rows_since_any = 0
        elif rx is not None:
            R[y] = rx
            stat["rows_right_only"] += 1
            rows_since_any = 0
        else:
            stat["rows_none"] += 1
            rows_since_any += 1
        if rows_since_any > max_gap_rows and last_both is not None:
            break  # 선이 끝났다
    return L, R, stat


def interpolate_edges(L, R, max_gap_rows):
    """점선 사이처럼 한쪽만 빈 행을 위아래 값으로 선형 보간. 양 끝 밖은 채우지 않는다."""
    def fill(arr):
        idx = np.flatnonzero(~np.isnan(arr))
        if idx.size < 2:
            return arr
        out = arr.copy()
        for a, b in zip(idx[:-1], idx[1:]):
            if b - a > 1 and (b - a) <= max_gap_rows:
                out[a + 1:b] = np.interp(np.arange(a + 1, b), [a, b], [arr[a], arr[b]])
        return out
    return fill(L), fill(R)


def lane_polygon(L, R, min_rows, epsilon=2.0):
    """L, R가 모두 있는 연속 행 구간을 폴리곤으로. 픽셀 좌표 리스트 또는 None."""
    both = ~np.isnan(L) & ~np.isnan(R)
    if both.sum() < min_rows:
        return None
    ys = np.flatnonzero(both)
    # 가장 아래에서 시작하는 연속 구간만 사용
    ys = ys[::-1]  # 아래(큰 y)부터
    seg = [int(ys[0])]
    for y in ys[1:]:
        if seg[-1] - y == 1:
            seg.append(int(y))
        else:
            break
    if len(seg) < min_rows:
        return None
    left = [(float(L[y]), float(y)) for y in seg]            # 아래→위
    right = [(float(R[y]), float(y)) for y in reversed(seg)]  # 위→아래
    pts = np.array(left + right, np.float32).reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(pts, epsilon, True).reshape(-1, 2)
    if len(approx) < 3:
        return None
    return [(float(x), float(y)) for x, y in approx], (seg[-1], seg[0])


def process_image(label_path, img_w, img_h, boundary_ids, args):
    items = read_label(label_path)
    polys = [pts for cls, pts in items if cls in boundary_ids]
    if not polys:
        return None, {"reason": "no_boundary_label"}
    mask = polygon_to_mask(polys, img_w, img_h)
    min_width = int(args.min_width * img_w)
    max_width = int(args.max_width * img_w)
    max_gap = int(args.max_gap * img_h)
    L, R, stat = scan_lane(mask, min_width, max_width, max_gap)
    L, R = interpolate_edges(L, R, max_gap)
    res = lane_polygon(L, R, min_rows=int(args.min_rows * img_h))
    if res is None:
        reason = "one_side_only" if (stat["rows_left_only"] + stat["rows_right_only"]) > stat["rows_both"] else "too_short"
        return None, {"reason": reason, **stat}
    poly, (top, bottom) = res
    norm = [(x / img_w, y / img_h) for x, y in poly]
    return norm, {"reason": "ok", "top_y": top, "bottom_y": bottom, **stat}


def overlay(img, mask, poly_px):
    vis = img.copy()
    blue = np.zeros_like(vis)
    blue[:, :, 0] = mask
    vis = cv2.addWeighted(vis, 1.0, blue, 0.6, 0)
    if poly_px is not None:
        fill = np.zeros_like(vis)
        cv2.fillPoly(fill, [np.array(poly_px, np.int32)], (0, 200, 0))
        vis = cv2.addWeighted(vis, 1.0, fill, 0.35, 0)
        cv2.polylines(vis, [np.array(poly_px, np.int32)], True, (0, 255, 0), 1)
    return vis


# --------------------------------------------------------------------- 실행
def convert(src, dst, args):
    data_yaml = os.path.join(src, "data.yaml")
    if not os.path.isfile(data_yaml):
        sys.exit("data.yaml 없음: %s" % data_yaml)
    d, names = read_names(data_yaml)
    boundary_ids = {names.index(n) for n in args.boundary if n in names}
    missing = [n for n in args.boundary if n not in names]
    if missing:
        sys.exit("경계 클래스가 data.yaml에 없음: %s (있는 이름: %s)" % (missing, names))
    if LANE_NAME in names:
        lane_id = names.index(LANE_NAME)
        new_names = names
    else:
        lane_id = len(names)
        new_names = names + [LANE_NAME]
    os.makedirs(dst, exist_ok=True)
    qa_dir = os.path.join(dst, "qa")
    os.makedirs(qa_dir, exist_ok=True)
    report = {"src": src, "dst": dst, "lane_class_id": lane_id, "boundary": sorted(boundary_ids), "splits": {}}
    qa_pool = []
    for split in ("train", "valid", "test"):
        img_dir = os.path.join(src, split, "images")
        lbl_dir = os.path.join(src, split, "labels")
        if not os.path.isdir(img_dir):
            continue
        out_img = os.path.join(dst, split, "images")
        out_lbl = os.path.join(dst, split, "labels")
        os.makedirs(out_img, exist_ok=True)
        os.makedirs(out_lbl, exist_ok=True)
        cnt = {"images": 0, "ok": 0, "one_side_only": 0, "too_short": 0, "no_boundary_label": 0, "unreadable": 0}
        skipped = []
        for fn in sorted(os.listdir(img_dir)):
            stem, ext = os.path.splitext(fn)
            if ext.lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
                continue
            cnt["images"] += 1
            img = cv2.imread(os.path.join(img_dir, fn))
            if img is None:
                cnt["unreadable"] += 1
                continue
            h, w = img.shape[:2]
            lbl = os.path.join(lbl_dir, stem + ".txt")
            poly, info = process_image(lbl, w, h, boundary_ids, args)
            src_lines = open(lbl, encoding="utf-8").read().rstrip("\n") if os.path.isfile(lbl) else ""
            src_lines = "\n".join(l for l in src_lines.splitlines() if not l.startswith("%d " % lane_id)) if LANE_NAME in names else src_lines
            if poly is not None:
                cnt["ok"] += 1
                line = "%d " % lane_id + " ".join("%.6f %.6f" % (x, y) for x, y in poly)
                out_text = (src_lines + "\n" if src_lines else "") + line + "\n"
            else:
                cnt[info["reason"]] += 1
                skipped.append({"file": fn, **info})
                out_text = src_lines + ("\n" if src_lines else "")
            if args.skip_without_lane and poly is None:
                continue
            shutil.copy2(os.path.join(img_dir, fn), os.path.join(out_img, fn))
            open(os.path.join(out_lbl, stem + ".txt"), "w", encoding="utf-8").write(out_text)
            qa_pool.append((split, fn, os.path.join(img_dir, fn), lbl, poly))
        cnt["skipped_list"] = skipped[:200]
        report["splits"][split] = cnt
        print("%s: images %d, lane ok %d, one_side_only %d, too_short %d, no_label %d"
              % (split, cnt["images"], cnt["ok"], cnt["one_side_only"], cnt["too_short"], cnt["no_boundary_label"]))
    write_names(data_yaml, os.path.join(dst, "data.yaml"), d, new_names)
    for extra in ("README.roboflow.txt", "README.dataset.txt"):
        if os.path.isfile(os.path.join(src, extra)):
            shutil.copy2(os.path.join(src, extra), os.path.join(dst, extra))
    # QA 오버레이: 성공/실패 섞어서
    random.seed(0)
    random.shuffle(qa_pool)
    n_qa = 0
    for split, fn, img_path, lbl, poly in qa_pool:
        if n_qa >= args.qa:
            break
        img = cv2.imread(img_path)
        h, w = img.shape[:2]
        items = read_label(lbl)
        mask = polygon_to_mask([pts for cls, pts in items if cls in boundary_ids], w, h)
        poly_px = [(x * w, y * h) for x, y in poly] if poly else None
        cv2.imwrite(os.path.join(qa_dir, "%s_%s.png" % (split, os.path.splitext(fn)[0])), overlay(img, mask, poly_px))
        n_qa += 1
    json.dump(report, open(os.path.join(dst, "report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("data.yaml names:", new_names)
    print("QA 오버레이 %d장: %s" % (n_qa, qa_dir))
    print("report: %s" % os.path.join(dst, "report.json"))
    return report


# --------------------------------------------------------------------- 합성 시험
def make_synthetic(root, n=6, w=320, h=240):
    """오른쪽 실선 + 왼쪽 점선(조각 여러 개) 곡선 트랙과, 한쪽 선만 있는 이미지 하나."""
    names = ["solid_line", "dashed_line", "traffic_light_red"]
    for split in ("train", "valid"):
        os.makedirs(os.path.join(root, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(root, split, "labels"), exist_ok=True)
    rng = random.Random(1)
    for i in range(n):
        split = "train" if i < n - 2 else "valid"
        img = np.full((h, w, 3), 40, np.uint8)
        curve = rng.uniform(-0.0015, 0.0015)
        lines = []
        def xc(y, base):  # 곡선: 위로 갈수록 휨
            return base + curve * (h - y) ** 2
        # 오른쪽 실선 폴리곤 (폭 6px), 아래→위
        rpts = [(xc(y, 240) , y) for y in range(h, 60, -10)]
        solid = [(x - 3, y) for x, y in rpts] + [(x + 3, y) for x, y in reversed(rpts)]
        lines.append((0, solid))
        # 왼쪽 점선: 길이 24px, 간격 24px
        y = h
        while y > 70:
            y0, y1 = y, max(y - 24, 70)
            dash = [(xc(y0, 80) - 3, y0), (xc(y0, 80) + 3, y0), (xc(y1, 80) + 3, y1), (xc(y1, 80) - 3, y1)]
            lines.append((1, dash))
            y -= 48
        if i == n - 1:  # 한쪽 선만
            lines = [l for l in lines if l[0] == 0]
        for cls, pts in lines:
            cv2.fillPoly(img, [np.array(pts, np.int32)], (255, 255, 255))
        fn = "syn_%02d" % i
        cv2.imwrite(os.path.join(root, split, "images", fn + ".jpg"), img)
        with open(os.path.join(root, split, "labels", fn + ".txt"), "w", encoding="utf-8") as f:
            for cls, pts in lines:
                f.write("%d " % cls + " ".join("%.6f %.6f" % (x / w, y / h) for x, y in pts) + "\n")
    open(os.path.join(root, "data.yaml"), "w", encoding="utf-8").write(
        "train: ../train/images\nval: ../valid/images\nnc: 3\nnames: [%s]\n" % ", ".join("'%s'" % n for n in names))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", nargs="?", help="Roboflow YOLOv8 세그 내보내기 폴더")
    ap.add_argument("dst", help="출력 폴더")
    ap.add_argument("--boundary", nargs="+", default=["solid_line", "dashed_line"], help="차로 경계로 쓸 클래스 이름")
    ap.add_argument("--min-width", type=float, default=0.10, help="차로 최소 폭, 이미지 폭 비율")
    ap.add_argument("--max-width", type=float, default=0.90, help="차로 최대 폭, 이미지 폭 비율")
    ap.add_argument("--max-gap", type=float, default=0.25, help="점선 사이 보간 최대 간격, 이미지 높이 비율")
    ap.add_argument("--min-rows", type=float, default=0.15, help="lane으로 인정할 최소 행 수, 이미지 높이 비율")
    ap.add_argument("--qa", type=int, default=40, help="확인용 오버레이 장수")
    ap.add_argument("--skip-without-lane", action="store_true", help="lane을 못 만든 이미지는 출력에서 뺀다")
    ap.add_argument("--selftest", action="store_true", help="합성 데이터로 동작 확인")
    args = ap.parse_args()
    if args.selftest:
        src = os.path.join(args.dst, "_synthetic_src")
        if os.path.isdir(src):
            shutil.rmtree(src)
        make_synthetic(src)
        rep = convert(src, os.path.join(args.dst, "_synthetic_out"), args)
        tr = rep["splits"]["train"]
        va = rep["splits"]["valid"]
        ok = tr["ok"] == tr["images"] and va["ok"] == va["images"] - 1 and va["one_side_only"] == 1
        print("SELFTEST", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    if not args.src:
        ap.error("src 폴더가 필요합니다")
    convert(args.src, args.dst, args)


if __name__ == "__main__":
    main()
