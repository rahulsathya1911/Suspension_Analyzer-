import json
import math
from pathlib import Path

ts_dir = Path("../../out/timeseries")
names = [
    "rA_v20_m440_c1800_d0_s0",
    "rB_v20_m440_c1800_d0_s0",
    "rC_v20_m440_c1800_d0_s0",
]


def rms(x):
    x = [v for v in x if isinstance(v, (int, float))]
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else float("nan")


def is_num_list(v):
    return isinstance(v, list) and v and isinstance(v[0], (int, float))


for n in names:
    d = json.load(open(ts_dir / f"{n}.json"))
    print("==", n)
    print("keys:", list(d.keys()))
    for k, v in d.items():
        if is_num_list(v):
            print(f"  {k:24s} len={len(v):6d} rms={rms(v):.6g}")
        elif isinstance(v, dict):
            for kk, vv in v.items():
                if is_num_list(vv):
                    print(f"  {k}.{kk:20s} len={len(vv):6d} rms={rms(vv):.6g}")