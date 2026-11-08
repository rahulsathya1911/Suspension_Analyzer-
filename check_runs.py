import json, urllib.request as u
B = "http://localhost:8000/fusion"
get = lambda p: json.load(u.urlopen(f"{B}/{p}"))
print(f"{'id':26} rej%   drop%  bv_rmse  st_fused  st_raw")
for i in get("scenarios")["timeseries"]:
    d = get(f"data/timeseries/{i}"); m = d["metrics"]
    rej = sum(d["flags"]["anomaly_rejected"]) / len(d["t"]) * 100
    print(f"{i:26} {rej:5.1f} {m['dropout_frac']*100:6.1f} "
          f"{m['rmse_fused']['body_vel_mps']:.4f}  "
          f"{m['rmse_fused']['susp_travel_m']:.5f}  {m['rmse_raw']['susp_travel_m']:.5f}")