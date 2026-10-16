# -*- coding: utf-8 -*-
"""
=====================================================================================
 차선 상태 필터 (lane_filter.py, 2026-10-16)  -  고스트 = "차로에 대해 차가 어디에, 어느 방향으로 있나" 를 계속 추정
=====================================================================================

H 까지는 사진마다 경로를 새로 만들고, 못 믿으면 직전 경로를 '그대로' 다시 썼다 (차가 움직인 것 무시).
이 필터는 상태를 하나 들고 다니면서
   예측: 조향각(가변저항 ADC)과 속도로 차가 움직인 만큼 상태를 옮긴다 (사진이 없어도 됨)
   갱신: 사진의 경계 점(점선 = 왼쪽, 실선 = 오른쪽)을 그대로 관측으로 넣는다 (중심선 평균, 폭 검사, hold 없음)
   불확실성 P: 관측이 없으면 커지고, 있으면 줄어든다. 이 값 하나가 hold/search/timeout 을 대신한다.

상태 (뒷바퀴 축 가운데 기준, vehicle.yaml 좌표를 x 방향으로 +0.79 m 옮긴 것)
   e     차로 중앙이 차보다 왼쪽에 있는 거리 (m, 차에서 가장 가까운 중앙선 점까지, 수직)
   psi   차 기준으로 본 그 점의 차로 방향 (rad, + = 차로가 왼쪽으로 향함)
   kap   그 점의 곡률 (1/m, + = 왼쪽 커브)
   dkap  곡률이 호 길이에 따라 변하는 비율 (1/m², 커브 들어갈 때/나올 때)
   차로 중앙선 = 그 점에서 시작해 방향 θ(s) = psi + kap·s + dkap·s²/2 로 그린 곡선 (반경 1.7 m 커브도 정확, 2차식 근사 아님)

운동 (Frenet, 뒷바퀴 축은 옆으로 미끄러지지 않는다고 봄)
   r = v·tan(δ)/L,  s' = v·cos(psi)/(1 + kap·e)
   e' = v·sin(psi),  psi' = kap·s' - r,  kap' = dkap·s'

ROS 와 무관한 순수 계산 모듈. 실차 노드/기록 재생(filter_replay.py) 양쪽에서 쓴다.
"""
import numpy as np

HALF_W = 0.425                    # 차로 폭 절반 (규격 0.85 m, 10/10 사진 중앙값 0.856~0.86)
S_GRID = np.arange(-1.0, 3.0001, 0.02)   # 중앙선을 그릴 호 길이 범위 (m)
I0 = int(np.argmin(np.abs(S_GRID)))


def centerline(x):
    """상태 → 중앙선 점들 (N,2) 과 각 점의 방향 θ (N,)."""
    e, psi, kap, dkap = x
    th = psi + kap * S_GRID + 0.5 * dkap * S_GRID ** 2
    ds = np.diff(S_GRID)
    tm = 0.5 * (th[1:] + th[:-1])                             # 구간 가운데 방향으로 적분
    steps = np.c_[np.cos(tm) * ds, np.sin(tm) * ds]
    P = np.zeros((len(S_GRID), 2))
    P[I0] = [-e * np.sin(psi), e * np.cos(psi)]               # 차에서 가장 가까운 중앙선 점
    P[I0 + 1:] = P[I0] + np.cumsum(steps[I0:], axis=0)
    P[:I0] = P[I0] - np.cumsum(steps[:I0][::-1], axis=0)[::-1]
    return P, th


def signed_dist(x, pts):
    """점들이 중앙선보다 왼쪽(+)으로 얼마나 떨어졌나, 가장 가까운 중앙선 점의 방향."""
    P, th = centerline(x)
    d2 = ((pts[:, None, :] - P[None, :, :]) ** 2).sum(-1)
    i = np.argmin(d2, axis=1)
    n = np.c_[-np.sin(th[i]), np.cos(th[i])]
    return ((pts - P[i]) * n).sum(1), th[i]


class LaneFilter:
    def __init__(self, wheelbase=0.56, sig_meas0=0.008, sig_meas_per_m=0.012):
        self.L = wheelbase
        self.sig0, self.sig1 = sig_meas0, sig_meas_per_m
        self.x = None; self.P = None
        # 과정 잡음 (단위 시간/거리당). 너무 작으면 커브 진입을 늦게 따라가고, 크면 사진마다 흔들린다
        self.q_e, self.q_psi, self.q_kap, self.q_dkap = 0.01, 0.15, 0.3, 3.0
        self.stats = {"pts_used": 0, "pts_gated": 0, "boundary_dropped": 0}

    # --- 예측: 차가 dt 동안 속도 v, 조향 δ 로 움직임 ---
    def predict(self, v, delta, dt):
        if self.x is None or dt <= 0:
            return
        e, psi, kap, dkap = self.x
        r = v * np.tan(delta) / self.L
        sdot = v * np.cos(psi) / max(1.0 + kap * e, 0.2)
        self.x = np.array([e + v * np.sin(psi) * dt, psi + (kap * sdot - r) * dt, kap + dkap * sdot * dt, dkap])
        F = np.eye(4)
        F[0, 1] = v * np.cos(psi) * dt
        F[1, 2] = sdot * dt; F[2, 3] = sdot * dt
        dist = abs(v) * dt
        Q = np.diag([self.q_e ** 2 * dt, (self.q_psi ** 2) * dt, self.q_kap ** 2 * dist + 1e-6, self.q_dkap ** 2 * dist + 1e-6])
        self.P = F @ self.P @ F.T + Q

    # --- 갱신: 경계 점 (뒷바퀴 축 기준 x, y). side = +1 점선(왼쪽 경계), -1 실선(오른쪽 경계) ---
    def update(self, pts_d, pts_s, max_pts=15):
        groups = []
        for pts, side in ((pts_d, +1.0), (pts_s, -1.0)):
            if pts is not None and len(pts) >= 4:
                k = np.linspace(0, len(pts) - 1, min(max_pts, len(pts))).astype(int)
                groups.append((np.asarray(pts, float)[k], side))
        if not groups:
            return False
        if self.x is None:
            return self._init(groups)
        z_all, H_all, R_all = [], [], []
        for pts, side in groups:
            res, H, sig = self._residual(self.x, pts, side)
            S = np.einsum("ij,jk,ik->i", H, self.P, H) + sig ** 2
            ok = np.abs(res) < 4.0 * np.sqrt(S)                      # 점 하나씩 거르기 (마할라노비스 4σ)
            self.stats["pts_gated"] += int((~ok).sum())
            if ok.mean() < 0.5:                                       # 반 넘게 안 맞으면 그 선 자체를 잘못 고른 것
                self.stats["boundary_dropped"] += 1
                continue
            # 같은 선의 점들은 오차가 서로 묶여 있다 → 점 수만큼 너무 믿지 않게 잡음을 키움
            inflate = np.sqrt(ok.sum() / 4.0)
            z_all.append(-res[ok]); H_all.append(H[ok]); R_all.append((sig[ok] * inflate) ** 2)
        if not z_all:
            return False
        z = np.concatenate(z_all); H = np.vstack(H_all); R = np.diag(np.concatenate(R_all))
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ z
        IKH = np.eye(4) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T                    # Joseph 형식 (수치 안정)
        self.stats["pts_used"] += len(z)
        self.last_nis = float(z @ np.linalg.solve(S, z) / len(z))
        return True

    def _residual(self, x, pts, side):
        d, _ = signed_dist(x, pts)
        res = d - side * HALF_W
        H = np.zeros((len(pts), 4))
        for j, eps in enumerate((1e-4, 1e-4, 1e-3, 1e-2)):            # 수치 미분
            xp = x.copy(); xp[j] += eps
            H[:, j] = ((signed_dist(xp, pts)[0] - side * HALF_W) - res) / eps
        sig = self.sig0 + self.sig1 * np.clip(np.linalg.norm(pts, axis=1) - 0.8, 0, None)
        return res, H, sig

    def _init(self, groups):
        """첫 관측: 가우스-뉴턴으로 상태를 바로 맞추고, P 는 크게."""
        x = np.zeros(4)
        for _ in range(8):
            rs, Hs = [], []
            for pts, side in groups:
                r, H, _ = self._residual(x, pts, side); rs.append(r); Hs.append(H)
            r = np.concatenate(rs); H = np.vstack(Hs)
            dx = np.linalg.lstsq(H.T @ H + np.diag([1e-4, 1e-4, 1e-2, 1.0]), -H.T @ r, rcond=None)[0]
            x = x + dx
            if np.abs(dx).max() < 1e-5:
                break
        if not np.all(np.isfinite(x)) or np.sqrt(np.mean(r ** 2)) > 0.05:
            return False
        self.x = x; self.P = np.diag([0.05, 0.1, 0.5, 2.0]) ** 2
        return True

    # --- 제어기에 줄 값: 임의의 차 위의 점 (예: 앞바퀴 축) 기준 오차 ---
    def errors_at(self, px, py=0.0):
        """차 위 점 (px, py) [뒷바퀴 축 기준 m] 에서: 차로 중앙이 왼쪽에 있는 거리, 차로 방향(rad), 곡률."""
        if self.x is None:
            return None
        d, th = signed_dist(self.x, np.array([[px, py]]))
        P, ths = centerline(self.x)
        i = int(np.argmin(((P - [px, py]) ** 2).sum(1)))
        s = S_GRID[i]
        return -float(d[0]), float(th[0]), float(self.x[2] + self.x[3] * s)

    def sigma_e(self):
        return None if self.P is None else float(np.sqrt(self.P[0, 0]))
