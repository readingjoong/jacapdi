# -*- coding: utf-8 -*-
"""주행 튜닝 슬라이더 (car_prep.sh 가 띄움. 따로: python3 ~/jacapdi-docker/tune_gui.py, 글씨 크기: TUNE_UI_SCALE=2, 창 위치: GUI_POS=x,y)
슬라이더를 움직이면 tune/tune.json 에 저장되고, 컨테이너의 tunable_controller.py 가 0.05 초 안에 반영한다 (주행 중에도).
위쪽 상태 줄 = 제어기가 0.2 초마다 쓰는 tune/status.json. 다른 곳에서 tune.json 을 바꾸면 슬라이더도 따라간다."""
import json, os, time
import tkinter as tk
from tkinter import ttk, filedialog
import tkinter.font as tkfont

UI = float(os.environ.get("TUNE_UI_SCALE", "1.5"))
STATUS_WIDTH = 90          # 위쪽 상태 두 줄의 너비 (글자 수). 고정해야 값이 바뀔 때 창이 꿈틀거리지 않음
DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune")
TUNE, DEFAULT, STATUS, LANE = DIR + "/tune.json", DIR + "/tune_default.json", DIR + "/status.json", DIR + "/lane"

# (키, 이름, 최소, 최대, 간격, 설명)
SLIDERS = [
    ("stanley_gain", "Stanley 게인", 0.0, 3.0, 0.05, "차로 중심에서 벗어난 거리에 대한 반응. 크면 세게 복귀(흔들림 증가)"),
    ("heading_gain", "방향 게인", 0.0, 2.0, 0.05, "경로 방향과 차 방향 차이에 대한 반응. 크면 흔들림 감쇠·커브 진입 빠름"),
    ("vehicle_speed_mps", "속도 가정 (m/s)", 0.2, 3.0, 0.05, "Stanley 식의 속도. 크면 거리 오차 반응이 약해짐"),
    ("softening_speed_mps", "저속 완화 (m/s)", 0.05, 1.0, 0.05, "속도 가정에 더해지는 값 (저속에서 조향이 튀는 것 완화)"),
    ("curvature_gain", "곡률 보정", 0.0, 2.0, 0.05, "경로가 휘는 만큼 미리 조향 (커브 대응)"),
    ("front_axle_offset_m", "평가 지점 (m)", -0.8, 0.8, 0.02, "경로를 평가하는 앞뒤 위치 (0 = 차 앞 약 0.84 m). 2차선에서 씀"),
    ("front_axle_offset_m_lane1", "1차선 평가 지점 (m)", -0.8, 0.8, 0.02, "1차선에서 쓰는 평가 지점. 크면 급코너를 안쪽으로 질러감 (가제보: 0.36 안쪽 이탈, 0.1 이탈 없음)"),
    ("lane1_outer", "1차선 바깥 실선 기준", 0, 1, 1, "1 = 1차선에서 점선 대신 2차선 바깥 실선으로 차로 가운데를 잡음 (안쪽 실선 우선). 0 = 원래대로"),
    ("max_steer_deg", "최대 조향각 (deg)", 10, 40, 1, "이 각도 = 끝까지(100%). 실측 바퀴 최대 25°. 작을수록 더 많이 꺾음"),
    ("i_gain", "I 게인", 0.0, 1.0, 0.02, "직선에서 한쪽으로 계속 벗어나 있으면 점점 더 꺾음. 0 = 끔"),
    ("i_max_deg", "I 최대 (deg)", 0.0, 8.0, 0.5, "I 항이 더하는 조향각 최대치"),
    ("i_curv_max", "I 직선 기준 (1/m)", 0.0, 1.0, 0.05, "곡률이 이보다 작을 때만 I 항을 쌓음"),
    ("lane_timeout_sec", "경로 끊김 허용 (s)", 0.1, 1.0, 0.05, "이 시간 넘게 경로가 없으면 정지"),
    ("pwm", "뒷바퀴 PWM", 0, 255, 5, "뒷바퀴 모터 힘 (0~255)"),
    ("lane_change_sec", "차선 변경 시간 (s)", 0.5, 4.0, 0.1, "차선을 바꿀 때 옆으로 0.85 m 옮기는 데 걸리는 시간. 짧으면 급하게 꺾음"),
    ("obstacle_dist_m", "장애물 피하기 거리 (m)", 0.0, 3.0, 0.1, "내 차선에 선 차가 앞범퍼에서 이 거리 안에 보이면 옆 차선으로 변경. 0 = 끔"),
    ("light_stop_dist_m", "신호 정지 거리 (m)", 0.0, 2.5, 0.05, "빨간·노란불일 때 횡단보도 줄무늬 먼 쪽 끝이 앞범퍼에서 이 거리 안이면 정지, 초록불에 출발. 0 = 끔 (횡단보도 바로 앞 = 약 1.3)"),
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
        self.status = tk.StringVar(value="제어기 상태: 대기"); self.status2 = tk.StringVar(value="")
        sf = ttk.Frame(top); sf.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        for var in (self.status, self.status2):        # 글자 수가 바뀌어도 창 너비가 안 바뀌게 너비 고정
            ttk.Label(sf, textvariable=var, width=STATUS_WIDTH, anchor="w", font=("TkFixedFont", int(11 * UI), "bold")).pack(anchor="w")
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
        self.lane = tk.StringVar(); ttk.Label(lane, textvariable=self.lane, width=12, font=("TkDefaultFont", int(10 * UI), "bold")).pack(side="left", padx=8)
        btn = ttk.Frame(top); btn.grid(row=2 * len(SLIDERS) + 2, column=0, columnspan=3, pady=(8, 0), sticky="w")
        ttk.Button(btn, text="기본값", command=lambda: self.apply(load(DEFAULT))).pack(side="left")
        ttk.Button(btn, text="프리셋 저장", command=self.save_preset).pack(side="left", padx=6)
        ttk.Button(btn, text="프리셋 열기", command=self.open_preset).pack(side="left")
        self.saved = tk.StringVar(); ttk.Label(btn, textvariable=self.saved, width=32, foreground="#2e7a4e").pack(side="left", padx=10)
        self.poll()

    def fmt(self, k):
        v, lab, step = self.vars[k]
        lab.config(text=("%d" % round(v.get())) if step >= 1 else ("%.2f" % v.get()))

    def changed(self, k):
        self.fmt(k)
        if self.pending: self.root.after_cancel(self.pending)
        self.pending = self.root.after(150, self.write)       # 드래그 중엔 0.15 초에 한 번만 저장

    def collect(self):
        return {k: int(round(v.get())) if k in ("pwm", "lane1_outer") else round(float(v.get()), 3) for k, (v, _, _) in self.vars.items()}

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
                self.status.set("제어기 상태: 응답 없음 (car_prep.sh 실행했는지 확인)"); self.status2.set("")
            else:
                num = lambda v, f: "-" if v is None else f % v
                self.status.set("조향 %+4d%%  PWM %3d  경로 %s  c=%6s  b=%6s  이동 %6s  각도 %6s°  I %5s°" % (
                    s["steer"], s["pwm"], "OK " if s["valid"] else "없음", num(s["c"], "%.3f"), num(s["b"], "%.3f"),
                    num(s["shift"], "%.3f"), num(s["delta_deg"], "%.1f"), num(s["i_deg"], "%.2f")))
                self.status2.set("신호 %-5s  횡단보도 먼 끝 %5s m  평가 지점 %5s m (%s차선)  %s" % (
                    {"stop": "빨강/노랑", "go": "초록"}.get(s.get("light"), "-"), num(s.get("cross_far"), "%.2f"),
                    num(s.get("x0"), "%.2f"), s.get("lane", "-"), "[신호 정지]" if s.get("hold") else ""))
        except (OSError, ValueError, KeyError):
            self.status.set("제어기 상태: 대기 (status.json 없음)"); self.status2.set("")
        self.root.after(200, self.poll)


root = tk.Tk()
for _n in ("TkDefaultFont", "TkTextFont", "TkFixedFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont", "TkSmallCaptionFont", "TkIconFont", "TkTooltipFont"):
    _f = tkfont.nametofont(_n); _f.configure(size=int(round(_f.cget("size") * UI)))   # 기본 글꼴 전부 UI 배 (음수 = 픽셀 단위, 부호 유지)
App(root)
root.update_idletasks()                           # 처음 크기로 창을 고정 (안의 글자가 바뀌어도 창이 꿈틀거리지 않게), 자리는 path_viewer 바로 아래
root.geometry("%dx%d+%s+%s" % ((root.winfo_reqwidth() + 20, root.winfo_reqheight()) + tuple(os.environ.get("GUI_POS", "0,780").split(","))))
root.mainloop()
