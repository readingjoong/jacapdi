# -*- coding: utf-8 -*-
"""주행 튜닝 슬라이더 (호스트에서 실행: python3 ~/jacapdi-docker/tune_gui.py, 글씨 크기: TUNE_UI_SCALE=2)
슬라이더를 움직이면 tune/tune.json 에 저장되고, 컨테이너의 tunable_controller.py 가 0.05 초 안에 반영한다 (주행 중에도).
위쪽 상태 줄 = 제어기가 0.2 초마다 쓰는 tune/status.json. 다른 곳에서 tune.json 을 바꾸면 슬라이더도 따라간다."""
import json, os, time
import tkinter as tk
from tkinter import ttk, filedialog
import tkinter.font as tkfont

UI = float(os.environ.get("TUNE_UI_SCALE", "1.5"))
DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune")
TUNE, DEFAULT, STATUS, LANE = DIR + "/tune.json", DIR + "/tune_default.json", DIR + "/status.json", DIR + "/lane"

# (키, 이름, 최소, 최대, 간격, 설명)
SLIDERS = [
    ("stanley_gain", "Stanley 게인", 0.0, 3.0, 0.05, "차로 중심에서 벗어난 거리에 대한 반응. 크면 세게 복귀(흔들림 증가)"),
    ("heading_gain", "방향 게인", 0.0, 2.0, 0.05, "경로 방향과 차 방향 차이에 대한 반응. 크면 흔들림 감쇠·커브 진입 빠름"),
    ("vehicle_speed_mps", "속도 가정 (m/s)", 0.2, 3.0, 0.05, "Stanley 식의 속도. 크면 거리 오차 반응이 약해짐"),
    ("softening_speed_mps", "저속 완화 (m/s)", 0.05, 1.0, 0.05, "속도 가정에 더해지는 값 (저속에서 조향이 튀는 것 완화)"),
    ("curvature_gain", "곡률 보정", 0.0, 2.0, 0.05, "경로가 휘는 만큼 미리 조향 (커브 대응)"),
    ("front_axle_offset_m", "평가 지점 (m)", -0.8, 0.8, 0.02, "경로를 평가하는 앞뒤 위치 (0 = 차 앞 약 0.84 m)"),
    ("max_steer_deg", "최대 조향각 (deg)", 10, 40, 1, "이 각도 = 끝까지(100%). 실측 바퀴 최대 25°. 작을수록 더 많이 꺾음"),
    ("i_gain", "I 게인", 0.0, 1.0, 0.02, "직선에서 한쪽으로 계속 벗어나 있으면 점점 더 꺾음. 0 = 끔"),
    ("i_max_deg", "I 최대 (deg)", 0.0, 8.0, 0.5, "I 항이 더하는 조향각 최대치"),
    ("i_curv_max", "I 직선 기준 (1/m)", 0.0, 1.0, 0.05, "곡률이 이보다 작을 때만 I 항을 쌓음"),
    ("lane_timeout_sec", "경로 끊김 허용 (s)", 0.1, 1.0, 0.05, "이 시간 넘게 경로가 없으면 정지"),
    ("pwm", "뒷바퀴 PWM", 0, 255, 5, "뒷바퀴 모터 힘 (0~255)"),
    ("lane_change_sec", "차선 변경 시간 (s)", 0.5, 4.0, 0.1, "차선을 바꿀 때 옆으로 0.85 m 옮기는 데 걸리는 시간. 짧으면 급하게 꺾음"),
    ("obstacle_dist_m", "장애물 피하기 거리 (m)", 0.0, 3.0, 0.1, "내 차선에 선 차가 앞범퍼에서 이 거리 안에 보이면 옆 차선으로 변경. 0 = 끔"),
]


def load(path):
    with open(path) as f:
        return json.load(f)


class App:
    def __init__(self, root):
        self.root = root; root.title("자캡디 주행 튜닝")
        self.vals = {**load(DEFAULT), **(load(TUNE) if os.path.exists(TUNE) else {})}   # tune.json 에 없는 새 항목은 기본값
        self.vars = {}; self.pending = None; self.my_mtime = None
        top = ttk.Frame(root, padding=8); top.pack(fill="both", expand=True)
        self.status = tk.StringVar(value="제어기 상태: 대기")
        ttk.Label(top, textvariable=self.status, font=("TkFixedFont", int(11 * UI), "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        for i, (k, name, lo, hi, step, tip) in enumerate(SLIDERS):
            v = tk.DoubleVar(value=self.vals[k]); lab = ttk.Label(top, width=7)
            self.vars[k] = (v, lab, step)
            ttk.Label(top, text=name, width=20).grid(row=2 * i + 1, column=0, sticky="w")
            tk.Scale(top, from_=lo, to=hi, resolution=step, orient="horizontal", length=int(360 * UI), variable=v, showvalue=False,
                     command=lambda _=None, k=k: self.changed(k)).grid(row=2 * i + 1, column=1)
            lab.grid(row=2 * i + 1, column=2); self.fmt(k)
            ttk.Label(top, text=tip, foreground="#666", font=("TkDefaultFont", int(8 * UI))).grid(row=2 * i + 2, column=0, columnspan=3, sticky="w")
        lane = ttk.Frame(top); lane.grid(row=2 * len(SLIDERS) + 1, column=0, columnspan=3, pady=(8, 0), sticky="w")
        ttk.Label(lane, text="차선 (주행 중 바로 바뀜, 키 1 / 2):").pack(side="left")
        for n in (1, 2):
            ttk.Button(lane, text="%d차선" % n, command=lambda n=n: self.set_lane(n)).pack(side="left", padx=4)
        root.bind("1", lambda e: self.set_lane(1)); root.bind("2", lambda e: self.set_lane(2))   # 이 창에서 1 / 2 키
        self.lane = tk.StringVar(); ttk.Label(lane, textvariable=self.lane, font=("TkDefaultFont", int(10 * UI), "bold")).pack(side="left", padx=8)
        btn = ttk.Frame(top); btn.grid(row=2 * len(SLIDERS) + 2, column=0, columnspan=3, pady=(8, 0), sticky="w")
        ttk.Button(btn, text="기본값", command=lambda: self.apply(load(DEFAULT))).pack(side="left")
        ttk.Button(btn, text="프리셋 저장", command=self.save_preset).pack(side="left", padx=6)
        ttk.Button(btn, text="프리셋 열기", command=self.open_preset).pack(side="left")
        self.saved = tk.StringVar(); ttk.Label(btn, textvariable=self.saved, foreground="#2e7a4e").pack(side="left", padx=10)
        self.poll()

    def fmt(self, k):
        v, lab, step = self.vars[k]
        lab.config(text=("%d" % round(v.get())) if step >= 1 else ("%.2f" % v.get()))

    def changed(self, k):
        self.fmt(k)
        if self.pending: self.root.after_cancel(self.pending)
        self.pending = self.root.after(150, self.write)       # 드래그 중엔 0.15 초에 한 번만 저장

    def collect(self):
        return {k: int(round(v.get())) if k == "pwm" else round(float(v.get()), 3) for k, (v, _, _) in self.vars.items()}

    def write(self):
        self.pending = None; self.vals = self.collect()
        with open(TUNE + ".tmp", "w") as f: json.dump(self.vals, f, indent=1, ensure_ascii=False)
        os.replace(TUNE + ".tmp", TUNE)
        self.my_mtime = os.path.getmtime(TUNE)                # 내가 쓴 시각 (밖에서 바뀐 것과 구분)
        self.saved.set("저장됨 %s" % time.strftime("%H:%M:%S"))

    def set_all(self, d):
        for k, (v, _, _) in self.vars.items():
            if k in d: v.set(d[k]); self.fmt(k)

    def apply(self, d):
        self.set_all(d); self.write()

    def set_lane(self, n):
        with open(LANE + ".tmp", "w") as f: f.write("%d\n" % n)
        os.replace(LANE + ".tmp", LANE)

    def save_preset(self):
        p = filedialog.asksaveasfilename(initialdir=DIR, initialfile="preset_%s.json" % time.strftime("%m%d_%H%M"), defaultextension=".json")
        if p:
            with open(p, "w") as f: json.dump(self.collect(), f, indent=1, ensure_ascii=False)
            self.saved.set("프리셋 저장: %s" % os.path.basename(p))

    def open_preset(self):
        p = filedialog.askopenfilename(initialdir=DIR, filetypes=[("json", "*.json")])
        if p: self.apply(load(p))

    def poll(self):
        try:                                                  # 다른 곳에서 tune.json 을 바꿨으면 슬라이더도 맞춘다
            mt = os.path.getmtime(TUNE)
            if not self.pending and mt != self.my_mtime:
                self.vals = load(TUNE); self.my_mtime = mt; self.set_all(self.vals)
                self.saved.set("파일에서 다시 읽음 %s" % time.strftime("%H:%M:%S"))
        except (OSError, ValueError):
            pass
        try:
            with open(LANE) as f: self.lane.set("지금: %s차선" % f.read().strip())
        except OSError:
            self.lane.set("지금: 2차선")
        try:
            s = load(STATUS)
            if time.time() - s["time"] > 2:
                self.status.set("제어기 상태: 응답 없음 (car_prep.sh 실행했는지 확인)")
            else:
                self.status.set("조향 %+d%%  PWM %3d  경로 %s  c=%s  b=%s  이동 %s  각도 %s°  I %s°" % (
                    s["steer"], s["pwm"], "OK " if s["valid"] else "없음", s["c"], s["b"], s["shift"], s["delta_deg"], s["i_deg"]))
        except (OSError, ValueError, KeyError):
            self.status.set("제어기 상태: 대기 (status.json 없음)")
        self.root.after(200, self.poll)


root = tk.Tk()
for _n in ("TkDefaultFont", "TkTextFont", "TkFixedFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont", "TkTooltipFont"):
    _f = tkfont.nametofont(_n); _f.configure(size=int(round(_f.cget("size") * UI)))   # 기본 글꼴 전부 UI 배 (음수 = 픽셀 단위, 부호 유지)
App(root); root.mainloop()
