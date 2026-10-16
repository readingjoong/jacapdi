# -*- coding: utf-8 -*-
"""자캡디 주행 튜닝 슬라이더 (노트북에서 실행: python3 ~/jacapdi-docker/tune_gui.py)
슬라이더를 움직이면 ~/jacapdi-docker/tune/tune.json 에 바로 저장되고, 컨테이너의 tunable_controller.py 가 0.05초 안에 반영한다.
주행 중에도 바꿀 수 있다. 위쪽 상태 줄은 제어기가 0.2초마다 남기는 status.json 을 보여준다."""
import json, os, time
import tkinter as tk
from tkinter import ttk, filedialog

DIR = os.path.expanduser("~/jacapdi-docker/tune")
TUNE, DEFAULT, STATUS = DIR + "/tune.json", DIR + "/tune_default.json", DIR + "/status.json"

# (키, 이름, 최소, 최대, 간격, 설명)
CTRL = [
    ("stanley_gain", "Stanley 게인 / Stanley gain", 0.0, 3.0, 0.05, "차로 중심에서 벗어난 거리에 대한 반응. 크면 세게 복귀(흔들림 증가)"),
    ("heading_gain", "방향 게인 / Heading gain", 0.0, 2.0, 0.05, "경로 방향과 차 방향 차이에 대한 반응. 크면 흔들림 감쇠·커브 진입 빠름"),
    ("vehicle_speed_mps", "속도 가정 / Assumed speed (m/s)", 0.2, 3.0, 0.05, "Stanley 식의 속도. 크면 거리 오차 반응이 약해짐"),
    ("curvature_gain", "곡률 보정 / Curvature gain", 0.0, 2.0, 0.05, "경로가 휘는 만큼 미리 조향 (커브 대응)"),
    ("softening_speed_mps", "저속 완화 / Softening speed", 0.05, 1.0, 0.05, "저속에서 조향이 튀는 것 완화"),
    ("front_axle_offset_m", "평가 지점 / Look-ahead offset (m)", -0.8, 0.8, 0.02, "경로를 평가하는 앞뒤 위치 (0 = 카메라 보정 근거리 끝)"),
    ("max_steer_deg", "최대 조향각 / Max steer angle (deg)", 10, 40, 1, "이 각도 = 7단계. 작을수록 더 많이 꺾음, 클수록 덜 꺾음 / Smaller = steers more"),
    ("max_steering_step", "최대 조향 단계 / Max steer step", 1, 7, 1, "펌웨어로 보내는 조향 단계 상한"),
    ("steer_trim", "조향 트림 / Steer trim (step)", -3, 3, 1, "직진이 한쪽으로 흐를 때 보정 (+ 오른쪽)"),
    ("path_offset_m", "경로 좌우 이동 / Path offset (m)", -0.3, 0.3, 0.01, "+ 이면 경로를 왼쪽으로 (오른쪽 실선 침범 시)"),
    ("inside_bias_m", "인코스 / Inside bias (m)", -0.2, 0.2, 0.01, "+ 커브 안쪽으로, - 바깥쪽으로. 커브에서 안쪽 점선 밟으면 - 로 / + inside, - outside"),
    ("inside_bias_right_m", "인코스-오른쪽 커브 / Inside bias right (m)", -0.2, 0.2, 0.01, "0 = 위와 같게. - 면 오른쪽 커브를 늦게·크게 돎 / - = later, wider right turns"),
    ("steer_rate", "급발진 방지 / Steer rate limit (step/0.05s)", 0, 7, 1, "조향을 한 번에 몇 단계까지 바꿀지. 1 = 부드럽게, 0 = 제한 없음"),
    ("motor_balance", "좌우 모터 보정 / Motor balance (PWM)", -30, 30, 1, "직선에서 오른쪽으로 흐르면 + (왼쪽 모터 줄임), 왼쪽으로 흐르면 -"),
    ("stall_boost_pwm", "멈춤 탈출 / Stall boost (PWM)", 0, 255, 5, "1초 동안 차가 안 움직이면 0.5초 이 PWM 으로. 0 = 끔 / 0 = off"),
    ("lane_guard_m", "선 넘기 방지-오른쪽 / Lane guard right (m)", 0.0, 0.3, 0.01, "실선 쪽으로 이만큼 벗어나면 오른쪽으로 더 못 꺾음. 0 = 끔"),
    ("lane_guard_push", "선 넘기 방지-되돌리기 / Lane guard push (step)", 0, 4, 1, "기준 넘으면 반대쪽으로 최소 이 단계. 0 = 막기만 / 0 = block only"),
    ("lane_guard_left_m", "선 넘기 방지-왼쪽 / Lane guard left (m)", 0.0, 0.3, 0.01, "점선 쪽 기준. 0 = 오른쪽과 같게. 인코스 쓰면 오른쪽보다 크게 (0.18)"),
    ("lane_timeout_sec", "경로 끊김 허용 / Path timeout (s)", 0.1, 1.0, 0.05, "이 시간 넘게 경로 없으면 정지"),
]

def load(path):
    with open(path) as f: return json.load(f)

class App:
    def __init__(self, root):
        self.root = root; root.title("자캡디 주행 튜닝 / Jacapdi drive tuning")
        self.vals = load(TUNE) if os.path.exists(TUNE) else load(DEFAULT)
        self.vals.setdefault("inside_bias_m", 0.0)
        self.vals.setdefault("steer_rate", 0)
        self.vals.setdefault("lane_guard_m", 0.0)
        self.vals.setdefault("motor_balance", 0)
        self.vals.setdefault("stall_boost_pwm", 0)
        self.vals.setdefault("lane_guard_left_m", 0.0)
        self.vals.setdefault("lane_guard_push", 0)
        self.vals.setdefault("inside_bias_right_m", 0.0)
        self.vars = {}; self.pwm_vars = []; self.pending = None; self.labels = []
        top = ttk.Frame(root, padding=8); top.pack(fill="both", expand=True)
        self.status = tk.StringVar(value="제어기 상태: 대기")
        ttk.Label(top, textvariable=self.status, font=("TkFixedFont", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        box = ttk.LabelFrame(top, text="조향 / Steering (Stanley)", padding=6); box.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        for i, (k, name, lo, hi, step, tip) in enumerate(CTRL):
            v = tk.DoubleVar(value=self.vals[k]); self.vars[k] = (v, step)
            ttk.Label(box, text=name, width=34).grid(row=2 * i, column=0, sticky="w")
            val = ttk.Label(box, width=7); val.grid(row=2 * i, column=2)
            s = tk.Scale(box, from_=lo, to=hi, resolution=step, orient="horizontal", length=320, variable=v, showvalue=False,
                         command=lambda _=None, v=v, val=val, st=step: self.changed(v, val, st))
            s.grid(row=2 * i, column=1); self.fmt(v, val, step); self.labels.append((v, val, step))
            ttk.Label(box, text=tip, foreground="#666", font=("TkDefaultFont", 8)).grid(row=2 * i + 1, column=0, columnspan=3, sticky="w")

        pw = ttk.LabelFrame(top, text="조향 단계별 뒷바퀴 PWM / Rear PWM per steer step", padding=6); pw.grid(row=1, column=1, sticky="nsew")
        for i in range(8):
            v = tk.IntVar(value=int(self.vals["pwm_by_step"][i])); self.pwm_vars.append(v)
            ttk.Label(pw, text="단계 Step %d%s" % (i, " (직진 straight)" if i == 0 else (" (최대 max)" if i == 7 else "")), width=24).grid(row=i, column=0, sticky="w")
            val = ttk.Label(pw, width=5); val.grid(row=i, column=2)
            tk.Scale(pw, from_=0, to=255, resolution=5, orient="horizontal", length=260, variable=v, showvalue=False,
                     command=lambda _=None, v=v, val=val: self.changed(v, val, 1)).grid(row=i, column=1)
            self.fmt(v, val, 1); self.labels.append((v, val, 1))
        self.all_pwm = tk.IntVar(value=int(self.vals["pwm_by_step"][0]))
        row = ttk.Frame(pw); row.grid(row=9, column=0, columnspan=3, pady=(8, 0), sticky="w")
        ttk.Label(row, text="전체 / All:").pack(side="left")
        ttk.Spinbox(row, from_=0, to=255, increment=10, textvariable=self.all_pwm, width=5).pack(side="left", padx=4)
        ttk.Button(row, text="모든 단계에 적용 / Apply to all", command=self.set_all).pack(side="left")
        ttk.Button(row, text="커브 감속 예시 / Slow in curves", command=self.curve_preset).pack(side="left", padx=4)

        btn = ttk.Frame(top); btn.grid(row=2, column=0, columnspan=2, pady=(8, 0), sticky="w")
        ttk.Button(btn, text="기본값 / Default", command=self.reset).pack(side="left")
        ttk.Button(btn, text="프리셋 저장 / Save preset", command=self.save_preset).pack(side="left", padx=6)
        ttk.Button(btn, text="프리셋 열기 / Open preset", command=self.open_preset).pack(side="left")
        ttk.Button(btn, text="커브 적극 / Curve aggressive", command=lambda: self.apply(load(DIR + "/preset_curve_aggressive.json"))).pack(side="left", padx=6)
        ttk.Button(btn, text="커브 중간 / Curve mid", command=lambda: self.apply(load(DIR + "/preset_curve_mid.json"))).pack(side="left")
        self.saved = tk.StringVar(value=""); ttk.Label(btn, textvariable=self.saved, foreground="#2e7a4e").pack(side="left", padx=10)
        self.poll()

    def fmt(self, v, lab, step):
        x = v.get(); lab.config(text=("%d" % round(x)) if step >= 1 else ("%.2f" % x))

    def changed(self, v, lab, step):
        self.fmt(v, lab, step)
        if self.pending: self.root.after_cancel(self.pending)
        self.pending = self.root.after(150, self.write)      # 드래그 중엔 0.15초마다 한 번만 저장

    def collect(self):
        d = dict(self.vals)
        for k, (v, step) in self.vars.items():
            d[k] = int(round(v.get())) if step >= 1 and k in ("max_steering_step", "steer_trim", "steer_rate", "motor_balance", "stall_boost_pwm", "lane_guard_push") else round(float(v.get()), 3)
        d["pwm_by_step"] = [int(v.get()) for v in self.pwm_vars]
        return d

    def write(self):
        self.pending = None; d = self.collect(); self.vals = d
        tmp = TUNE + ".tmp"
        with open(tmp, "w") as f: json.dump(d, f, indent=1, ensure_ascii=False)
        os.replace(tmp, TUNE)
        self.my_mtime = os.path.getmtime(TUNE)               # 내가 쓴 파일 시각 (밖에서 바뀐 것과 구분)
        self.saved.set("저장됨 %s" % time.strftime("%H:%M:%S"))

    def apply(self, d):
        for k, (v, _) in self.vars.items(): v.set(d[k])
        for v, x in zip(self.pwm_vars, d["pwm_by_step"]): v.set(int(x))
        self.refresh_labels(); self.write()

    def refresh_labels(self):
        for v, lab, step in self.labels: self.fmt(v, lab, step)

    def set_all(self):
        for v in self.pwm_vars: v.set(int(self.all_pwm.get()))
        self.refresh_labels(); self.write()

    def curve_preset(self):
        base = int(self.all_pwm.get())
        for i, v in enumerate(self.pwm_vars): v.set(int(base if i <= 3 else base * (1 - 0.1 * (i - 3))))
        self.refresh_labels(); self.write()

    def reset(self): self.apply(load(DEFAULT))

    def save_preset(self):
        p = filedialog.asksaveasfilename(initialdir=DIR, initialfile="preset_%s.json" % time.strftime("%m%d_%H%M"), defaultextension=".json")
        if p:
            with open(p, "w") as f: json.dump(self.collect(), f, indent=1, ensure_ascii=False)
            self.saved.set("프리셋 저장: %s" % os.path.basename(p))

    def open_preset(self):
        p = filedialog.askopenfilename(initialdir=DIR, filetypes=[("json", "*.json")])
        if p: self.apply(load(p))

    def sync_from_file(self):
        # 다른 곳(Claude, 다른 창)에서 tune.json 을 바꿨으면 슬라이더도 그 값으로 맞춘다 (덮어쓰기 방지)
        try:
            mt = os.path.getmtime(TUNE)
            if self.pending or mt == getattr(self, "my_mtime", None):
                return
            d = load(TUNE)
        except (OSError, ValueError):
            return
        self.my_mtime = mt; self.vals = d
        for k, (v, _) in self.vars.items():
            if k in d: v.set(d[k])
        for v, x in zip(self.pwm_vars, d["pwm_by_step"]): v.set(int(x))
        self.refresh_labels()
        self.saved.set("파일에서 다시 읽음 %s" % time.strftime("%H:%M:%S"))

    def poll(self):
        self.sync_from_file()
        try:
            s = load(STATUS); age = time.time() - s["time"]
            if age > 2: self.status.set("제어기 상태: 응답 없음 (car_prep.sh 실행했는지 확인)")
            else:
                self.status.set("조향 %+d  PWM %3d  경로 %s  c=%s  b=%s  이동 %s  각도 %s°" % (
                    s["steer"], s["pwm"], "OK " if s["valid"] else "없음", s["c"], s["b"], s.get("shift"), s["delta_deg"]))
        except (OSError, ValueError, KeyError):
            self.status.set("제어기 상태: 대기 (status.json 없음)")
        self.root.after(200, self.poll)

root = tk.Tk(); App(root); root.mainloop()
