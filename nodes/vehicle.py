# -*- coding: utf-8 -*-
"""차량 기하·속도 상수 로더 (ROS 없음). 단일 출처 = config/vehicle.yaml.

경로 노드·제어기가 같은 함수를 쓴다:
    import vehicle
    vehicle.path_origin_x_m()   # 뒷축 → IPM 원점 거리 = cam_to_rear_axle_m + ipm_x_near_m (1.63)
    vehicle.wheelbase_m()       # 축간거리 (0.56)
    vehicle.max_steer_deg()     # 최대 조향각 (25)
    vehicle.speed_mps(pwm)      # PWM → 속도 (표 선형 보간), vehicle.v255_mps() = speed_mps(255)

파일은 첫 호출에 한 번 읽어 캐시한다 (load(path, force=True) 로 다시 읽기). pyyaml 이 있으면 safe_load,
없으면 이 파일 형식(키: 값, 한 단계 중첩 매핑, # 주석)만 읽는 작은 파서로 대체.
"""
import os

VEHICLE_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "vehicle.yaml")
_cache = {}


def _scalar(s):
    s = s.strip()
    if s in ("true", "True"): return True
    if s in ("false", "False"): return False
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s.strip("'\"")


def _parse_minimal(text):
    """pyyaml 부재 시: 'key: value' 와 들여쓴 한 단계 매핑만 읽는다."""
    doc = {}; cur = None
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or ":" not in line:
            continue
        k, v = line.split(":", 1)
        if line.startswith((" ", "\t")):
            if cur is None: raise ValueError("들여쓴 줄의 상위 키 없음: %r" % raw)
            doc[cur][_scalar(k)] = _scalar(v)
        else:
            k = k.strip()
            if v.strip() == "":
                doc[k] = {}; cur = k
            else:
                doc[k] = _scalar(v); cur = None
    return doc


def load(path=VEHICLE_YAML, force=False):
    """vehicle.yaml → dict (캐시)."""
    if not force and path in _cache:
        return _cache[path]
    with open(path) as f:
        text = f.read()
    try:
        import yaml
        doc = yaml.safe_load(text)
    except ImportError:
        doc = _parse_minimal(text)
    for k in ("wheelbase_m", "cam_to_rear_axle_m", "ipm_x_near_m", "max_steer_deg", "pwm_speed_mps"):
        if k not in doc:
            raise KeyError("%s 에 %s 없음" % (path, k))
    _cache[path] = doc
    return doc


def wheelbase_m(path=VEHICLE_YAML):
    return float(load(path)["wheelbase_m"])


def cam_to_rear_axle_m(path=VEHICLE_YAML):
    return float(load(path)["cam_to_rear_axle_m"])


def ipm_x_near_m(path=VEHICLE_YAML):
    return float(load(path)["ipm_x_near_m"])


def path_origin_x_m(path=VEHICLE_YAML):
    """뒷축 → 경로(IPM) 원점 거리 m = cam_to_rear_axle_m + ipm_x_near_m."""
    return float(load(path)["cam_to_rear_axle_m"]) + float(load(path)["ipm_x_near_m"])


def max_steer_deg(path=VEHICLE_YAML):
    return float(load(path)["max_steer_deg"])


def speed_mps(pwm, path=VEHICLE_YAML):
    """PWM → 속도 m/s. 표 구간 선형 보간, 범위 밖은 끝값."""
    tab = sorted((int(k), float(v)) for k, v in load(path)["pwm_speed_mps"].items())
    if pwm <= tab[0][0]: return tab[0][1]
    if pwm >= tab[-1][0]: return tab[-1][1]
    for (p0, v0), (p1, v1) in zip(tab, tab[1:]):
        if p0 <= pwm <= p1:
            return v0 + (v1 - v0) * (pwm - p0) / (p1 - p0)
    return tab[-1][1]


def v255_mps(path=VEHICLE_YAML):
    """PWM 255 속도 m/s (속도 스케줄러 분모 눈금)."""
    return speed_mps(255, path)


if __name__ == "__main__":
    d = load()
    print("path_origin_x_m = %.4f  wheelbase_m = %.3f  max_steer_deg = %.1f  v255 = %.3f m/s  measured = %s"
          % (path_origin_x_m(), wheelbase_m(), max_steer_deg(), v255_mps(), d.get("measured")))
