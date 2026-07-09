#!/usr/bin/env python3
"""
Stage B of the PD gain designer.

Reads Meff.json (link-side effective inertia per joint, from export_Meff.py) and
an editable per-joint config, then for every joint:

  I_eff(q)  = M_ii(q) [URDF] + I_rotor [bench measurement]
  Kp        = torque / req_error            (your form; constant, posture-independent)
  Kd        = 2 * sqrt(Kp * I_eff_design)   (your form; critical-damping, design posture)
  omega_n(q)= sqrt(Kp / I_eff(q))           (resulting bandwidth, varies with posture)
  zeta(q)   = (B + Kd) / (2*sqrt(Kp*I_eff(q)))  (resulting damping, varies with posture)

Because Kp,Kd are fixed but I_eff swings with posture, omega_n and zeta are
reported across the Monte-Carlo posture envelope (min/median/max I_eff).

Runs in the RobStride venv (numpy/scipy/matplotlib):
    PY=/home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/venv/bin/python
    $PY pd_gain_designer.py --meff Meff.json --sim-joint shoulder_pitch_l --target-deg 20
"""
import argparse, json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-blocking: write PNG, never open a window
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ---- defaults applied to any joint not overridden in the config -------------
DEFAULTS = {
    "model": "RS04",
    "I_rotor": 0.030,      # kg m^2, measured (reflected to output)
    "B": 0.25,             # N m s/rad, measured viscous damping
    "torque_Nm": 4.5,      # commanded torque for the Kp = torque/req_error rule
    "req_error_deg": 10.0, # error at which that torque is commanded
}


def load_config(path, joints):
    if path and os.path.exists(path):
        with open(path) as f:
            cfg = json.load(f)
    else:
        cfg = {}
    full = {}
    for j in joints:
        d = dict(DEFAULTS)
        d.update(cfg.get(j, {}))
        full[j] = d
    if path and not os.path.exists(path):
        with open(path, "w") as f:
            json.dump(full, f, indent=2)
        print(f"[config] wrote editable default config -> {path}")
    return full


def design_point(jrec, which):
    return {"neutral": jrec["M_neutral"], "min": jrec["M_min"],
            "median": jrec["M_median"], "max": jrec["M_max"]}[which]


def step_metrics(t, y, target):
    y = np.asarray(y); yf = target
    # rise time 10->90%
    try:
        t10 = t[np.argmax(y >= 0.1 * yf)]
        t90 = t[np.argmax(y >= 0.9 * yf)]
        rise = max(t90 - t10, 0.0)
    except Exception:
        rise = float("nan")
    overshoot = max(0.0, (np.max(y) - yf) / yf * 100.0)
    # 2% settling
    outside = np.where(np.abs(y - yf) > 0.02 * abs(yf))[0]
    settle = t[outside[-1]] if len(outside) else 0.0
    return rise, overshoot, settle


def simulate(Kp, Kd, I, B, target_rad, tmax=2.0):
    c = B + Kd
    def f(t, x):
        th, w = x
        thdd = (Kp * (target_rad - th) - c * w) / I
        return [w, thdd]
    sol = solve_ivp(f, (0, tmax), [0.0, 0.0], max_step=tmax / 2000, rtol=1e-8, atol=1e-10)
    return sol.t, sol.y[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--meff", default="Meff.json")
    ap.add_argument("--config", default="pd_config.json")
    ap.add_argument("--design-point", default="median",
                    choices=["neutral", "min", "median", "max"],
                    help="posture I_eff used to fix Kd")
    ap.add_argument("--sim-joint", default="shoulder_pitch_l")
    ap.add_argument("--target-deg", type=float, default=20.0)
    ap.add_argument("--out-table", default="pd_gains.csv")
    ap.add_argument("--out-plot", default=None)
    args = ap.parse_args()

    with open(args.meff) as f:
        meff = json.load(f)
    joints = meff["joints"]
    cfg = load_config(args.config, list(joints.keys()))

    rows = []
    for j, m in joints.items():
        c = cfg[j]
        Irot, B = c["I_rotor"], c["B"]
        req = np.deg2rad(c["req_error_deg"])
        Kp = c["torque_Nm"] / req                       # constant
        Idesign = design_point(m, args.design_point) + Irot
        Kd = 2.0 * np.sqrt(Kp * Idesign)                # your formula at design posture
        # envelope of I_eff
        Ie = {k: m[f"M_{k}"] + Irot for k in ["min", "median", "max", "neutral"]}
        def wn(I): return np.sqrt(Kp / I)
        def zeta(I): return (B + Kd) / (2.0 * np.sqrt(Kp * I))
        rows.append({
            "joint": j, "model": c["model"], "Kp": Kp, "Kd": Kd,
            "I_eff_min": Ie["min"], "I_eff_med": Ie["median"], "I_eff_max": Ie["max"],
            "wn_min_Hz": wn(Ie["max"]) / (2*np.pi),   # max inertia -> min bandwidth
            "wn_max_Hz": wn(Ie["min"]) / (2*np.pi),
            "zeta_at_minI": zeta(Ie["min"]), "zeta_at_maxI": zeta(Ie["max"]),
        })

    # write table
    import csv
    with open(args.out_table, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 5) if isinstance(v, float) else v) for k, v in r.items()})
    print(f"[table] wrote {args.out_table} ({len(rows)} joints)\n")

    # console summary: joints with the widest zeta spread (most posture-sensitive)
    rows.sort(key=lambda r: abs(r["zeta_at_minI"] - r["zeta_at_maxI"]), reverse=True)
    print(f"{'joint':<20}{'Kp':>8}{'Kd':>8}{'wn[Hz] min..max':>18}{'zeta min..max':>18}")
    for r in rows[:14]:
        print(f"{r['joint']:<20}{r['Kp']:>8.2f}{r['Kd']:>8.3f}"
              f"{r['wn_min_Hz']:>8.2f}..{r['wn_max_Hz']:<8.2f}"
              f"{min(r['zeta_at_minI'],r['zeta_at_maxI']):>8.2f}..{max(r['zeta_at_minI'],r['zeta_at_maxI']):<8.2f}")

    # ---- simulate the chosen joint across its posture envelope -----------
    j = args.sim_joint
    if j not in joints:
        print(f"\n[sim] joint '{j}' not found; skipping plot")
        return
    m, c = joints[j], cfg[j]
    Irot, B = c["I_rotor"], c["B"]
    Kp = c["torque_Nm"] / np.deg2rad(c["req_error_deg"])
    Idesign = design_point(m, args.design_point) + Irot
    Kd = 2.0 * np.sqrt(Kp * Idesign)
    target = np.deg2rad(args.target_deg)

    fig, ax = plt.subplots(figsize=(9, 5))
    for label, key, col in [("tucked (min I)", "min", "tab:green"),
                            ("median I", "median", "tab:blue"),
                            ("extended (max I)", "max", "tab:red")]:
        I = m[f"M_{key}"] + Irot
        t, y = simulate(Kp, Kd, I, B, target)
        rise, ov, settle = step_metrics(t, np.rad2deg(y), args.target_deg)
        wn = np.sqrt(Kp / I) / (2*np.pi); z = (B + Kd) / (2*np.sqrt(Kp * I))
        ax.plot(t, np.rad2deg(y), col, lw=2,
                label=f"{label}: I={I:.3f}, wn={wn:.2f}Hz, z={z:.2f}, "
                      f"OS={ov:.0f}%, ts={settle:.2f}s")
    ax.axhline(args.target_deg, ls="--", c="k", alpha=0.4)
    ax.set(xlabel="time (s)", ylabel="angle (deg)",
           title=f"{j}: PD step (Kp={Kp:.1f}, Kd={Kd:.2f}) across posture envelope")
    ax.legend(fontsize=8, loc="lower right"); ax.grid(alpha=0.3)
    out = args.out_plot or f"pd_step_{j}.png"
    fig.tight_layout(); fig.savefig(out, dpi=120)
    print(f"\n[sim] wrote {out}")


if __name__ == "__main__":
    main()
