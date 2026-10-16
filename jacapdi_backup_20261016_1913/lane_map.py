# -*- coding: utf-8 -*-
"""
=====================================================================================
 고스트 + 기억 지도 (lane_map.py, 2026-10-16)  -  차가 지나갈 자리는 조금 전 사진에 이미 찍혀 있었다
=====================================================================================

카메라에 보이는 가장 가까운 곳은 앞범퍼 0.15~0.23 m 앞이다. 앞바퀴 축까지 0.4~0.5 m 는 어떤 사진에도 안 나온다.
lane_filter.py 처럼 차로를 곡선 하나(상태 4 개)로 표현해 뒤로 늘리면, 직선→원호 이음매에서 3~4° 가 오래 틀린다
(가상 트랙 시험). 그래서 곡선으로 추측하지 않고, 본 점을 기억한다.

   1) 고스트 위치: 조향각(ADC)과 속도로 차(뒷바퀴 축)의 위치/방향을 바닥 고정 좌표(오도메트리)에서 적분
   2) 사진마다: 경계 점(점선 = 왼쪽, 실선 = 오른쪽)을 그 순간 고스트 위치로 바닥 좌표에 옮겨 쌓는다
   3) 묻기: 지금 고스트 기준으로 앞바퀴 축 주변(뒤 0.4 m ~ 앞 0.8 m) 에 쌓인 점으로 경계를 짧게 맞추고
      차로 중앙의 옆 오차 / 방향 / 곡률을 낸다. 최근 점일수록 무겁게 (적분 오차가 덜 쌓였으니)
   오래되거나 차 뒤로 많이 지난 점은 버린다. 사진이 잠깐 없어도 쌓인 점과 고스트 이동만으로 답이 나온다
   (hold / search 가 따로 필요 없음). 답에 쓴 점이 너무 적으면 None = 정지 판단은 제어기가.

좌표: 차 기준은 뒷바퀴 축 가운데 (vehicle.yaml x + 0.79), x 앞, y 왼쪽, m
"""
import numpy as np

HALF_W = 0.425


class LaneMap:
    def __init__(self, wheelbase=0.56, keep_sec=2.0, keep_behind_m=0.6, tau_sec=0.6):
        self.L = wheelbase
        self.keep_sec, self.keep_behind, self.tau = keep_sec, keep_behind_m, tau_sec
        self.X = self.Y = self.TH = 0.0             # 고스트 (뒷바퀴 축) 위치/방향, 바닥 고정 좌표
        self.t = 0.0
        self.pts = np.zeros((0, 4))                  # 쌓인 점: X, Y, side(+1 점선 / -1 실선), 찍힌 시각

    # --- 고스트 이동 (자전거 모델, 뒷바퀴 축 기준) ---
    def move(self, v, delta, dt):
        n = max(1, int(np.ceil(dt / 0.01)))
        h = dt / n; r = v * np.tan(delta) / self.L
        for _ in range(n):
            self.TH += 0.5 * r * h
            self.X += v * np.cos(self.TH) * h; self.Y += v * np.sin(self.TH) * h
            self.TH += 0.5 * r * h
        self.t += dt

    def to_world(self, P):
        c, s = np.cos(self.TH), np.sin(self.TH)
        return np.c_[self.X + c * P[:, 0] - s * P[:, 1], self.Y + s * P[:, 0] + c * P[:, 1]]

    def to_car(self, W):
        c, s = np.cos(self.TH), np.sin(self.TH)
        dx, dy = W[:, 0] - self.X, W[:, 1] - self.Y
        return np.c_[c * dx + s * dy, -s * dx + c * dy]

    # --- 고스트 정합: 새 사진의 점을 기억한 점에 겹쳐 맞추고, 그만큼 고스트 위치를 바로잡는다 ---
    def match(self, pts_d, pts_s, trunc=0.05, sig=(0.05, 0.02, np.radians(2.0)), **_):
        """조향/속도로 옮긴 고스트는 출발점(사전)일 뿐. 겹치는 점이 충분할 때만 고친다. 고친 양 (dx, dy, dth) 을 돌려준다."""
        from scipy.optimize import minimize
        from scipy.spatial import cKDTree
        if not len(self.pts):
            return None
        groups = []
        for P, side in ((pts_d, 1.0), (pts_s, -1.0)):
            old = self.pts[self.pts[:, 2] == side]
            if P is None or len(P) < 8 or len(old) < 20:
                continue
            Oc = self.to_car(old[:, :2])
            groups.append((np.asarray(P, float), cKDTree(Oc), Oc[:, 0].min(), Oc[:, 0].max()))
        if not groups:
            return None

        def cost(p):
            dx, dy, dth = p; c, s = np.cos(dth), np.sin(dth); tot = n = ntot = 0
            for P, tree, lo, hi in groups:
                Q = np.c_[dx + c * P[:, 0] - s * P[:, 1], dy + s * P[:, 0] + c * P[:, 1]]   # 고친 고스트에서 본 새 점
                ntot += len(Q); Q = Q[(Q[:, 0] > lo + 0.03) & (Q[:, 0] < hi - 0.03)]
                if len(Q) < 5:
                    continue
                d, _ = tree.query(Q); tot += (np.minimum(d, trunc) ** 2).sum(); n += len(Q)
            if n < max(15, 0.25 * ntot):
                return 1e3
            return tot / n / 0.01 ** 2 + (dx / sig[0]) ** 2 + (dy / sig[1]) ** 2 + (dth / sig[2]) ** 2
        if cost([0, 0, 0]) >= 1e3:
            return None
        r = minimize(cost, [0.0, 0.0, 0.0], method="Powell", bounds=[(-0.15, 0.15), (-0.06, 0.06), (-0.12, 0.12)])
        if r.fun >= 1e3:
            return None
        dx, dy, dth = r.x
        # 고스트(뒷바퀴 축)를 차 기준 (dx, dy, dth) 만큼 옮긴다
        c, s = np.cos(self.TH), np.sin(self.TH)
        self.X += c * dx - s * dy; self.Y += s * dx + c * dy; self.TH += dth
        return dx, dy, dth

    # --- 사진 한 장의 경계 점 (차 기준) 을 쌓는다 (먼저 match 로 고스트를 맞춘 뒤) ---
    def add(self, pts_d, pts_s, match=True):
        self.last_match = self.match(pts_d, pts_s) if match else None
        for P, side in ((pts_d, 1.0), (pts_s, -1.0)):
            if P is not None and len(P):
                W = self.to_world(np.asarray(P, float))
                self.pts = np.vstack([self.pts, np.c_[W, np.full(len(W), side), np.full(len(W), self.t)]])
        # 오래된 점, 차 뒤로 많이 지난 점 버리기
        if len(self.pts):
            C = self.to_car(self.pts[:, :2])
            keep = (self.t - self.pts[:, 3] <= self.keep_sec) & (C[:, 0] >= -self.keep_behind)
            self.pts = self.pts[keep]

    # --- 차 위 x = px (예: 앞바퀴 축 0.56) 에서의 차로 오차 ---
    def errors_at(self, px, back=0.4, ahead=0.8, min_pts=6, ages=(None, 0.8, 0.4)):
        """ages: 모든 점으로 안 맞으면 최근 0.8 s, 그래도 안 되면 0.4 s 점만으로 다시 (커브에서 옛 점이 어긋나 쌓인 경우)."""
        if not len(self.pts):
            return None
        C = self.to_car(self.pts[:, :2])
        age = self.t - self.pts[:, 3]
        win = (C[:, 0] >= px - back) & (C[:, 0] <= px + ahead)
        w_age = np.exp(-age / self.tau)
        fits = {}; used_age = {}
        for side in (1.0, -1.0):
            for amax in ages:
                k = win & (self.pts[:, 2] == side)
                if amax is not None:
                    k &= age <= amax
                if k.sum() < min_pts or np.ptp(C[k, 0]) < 0.3:
                    continue
                x, y, wt = C[k, 0] - px, C[k, 1], w_age[k]
                coef = np.polyfit(x, y, 2, w=np.sqrt(wt))
                res = y - np.polyval(coef, x)
                if np.sqrt(np.average(res ** 2, weights=wt)) > 0.03:
                    continue
                fits[side] = coef; used_age[side] = amax
                break
        if not fits:
            return None
        # 차로 중앙 = 두 경계 가운데 (한쪽뿐이면 수직 0.425 m 를 기울기 보정해 옮김)
        cen = []
        for side, c in fits.items():
            m = c[1]
            cen.append(np.array([c[0], c[1], c[2] - side * HALF_W * np.sqrt(1 + m * m)]))
        c = np.mean(cen, axis=0)
        e, m, a = c[2], c[1], c[0]
        kap = 2 * a / (1 + m * m) ** 1.5
        width = None
        if len(fits) == 2:
            width = (fits[1.0][2] - fits[-1.0][2]) / np.sqrt(1 + (0.5 * (fits[1.0][1] + fits[-1.0][1])) ** 2)
        return {"e": float(e), "psi": float(np.arctan(m)), "kap": float(kap), "sides": len(fits), "width": width, "used_age": used_age,
                "n": int(win.sum()), "newest_age": float(self.t - self.pts[win, 3].max()) if win.any() else None}
