#!/usr/bin/env python3
"""
Stage A of the PD gain designer.

Loads the humanoid URDF with pinocchio and computes the *link-side* effective
joint inertia M_ii(q) (diagonal of the joint-space mass matrix) for every
actuated joint -- both at the neutral pose and across a Monte-Carlo sweep of
random postures within the joint limits.  Dumps the result to JSON for Stage B.

Run under the ROS python (matched numpy 1.26):
    source /opt/ros/jazzy/setup.bash
    PYTHONNOUSERSITE=1 python3 export_Meff.py \
        --urdf /home/xp/real_ws/src/full_body_mujoco/urdf/full_body.urdf \
        --out  Meff.json --mc 2000
"""
import argparse, json
import numpy as np
import pinocchio as pin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="/home/xp/real_ws/src/full_body_mujoco/urdf/full_body.urdf")
    ap.add_argument("--out", default="Meff.json")
    ap.add_argument("--mc", type=int, default=2000, help="Monte-Carlo posture samples")
    ap.add_argument("--clamp", type=float, default=np.pi,
                    help="bound (rad) used when a joint limit is non-finite")
    args = ap.parse_args()

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    nv = model.nv

    # clamped bounds for sampling
    lo = np.array(model.lowerPositionLimit, dtype=float).copy()
    hi = np.array(model.upperPositionLimit, dtype=float).copy()
    lo[~np.isfinite(lo)] = -args.clamp
    hi[~np.isfinite(hi)] = args.clamp
    # if a joint has lo==hi (locked) keep it; if inverted, swap
    swap = lo > hi
    lo[swap], hi[swap] = hi[swap], lo[swap]

    # neutral diagonal
    q0 = pin.neutral(model)
    M0 = np.array(pin.crba(model, data, q0))
    diag0 = np.diag(M0).copy()

    # Monte-Carlo envelope
    rng_min = diag0.copy()
    rng_max = diag0.copy()
    acc = np.zeros((args.mc, nv))
    for s in range(args.mc):
        q = pin.randomConfiguration(model, lo, hi)
        M = np.array(pin.crba(model, data, q))
        d = np.diag(M)
        acc[s] = d
        rng_min = np.minimum(rng_min, d)
        rng_max = np.maximum(rng_max, d)
    med = np.median(acc, axis=0)

    joints = {}
    for j in range(1, model.njoints):
        name = model.names[j]
        jt = model.joints[j].shortname()
        iv = model.joints[j].idx_v
        for k in range(model.joints[j].nv):
            i = iv + k
            joints[name if model.joints[j].nv == 1 else f"{name}_{k}"] = {
                "type": jt,
                "M_neutral": float(diag0[i]),
                "M_min": float(rng_min[i]),
                "M_median": float(med[i]),
                "M_max": float(rng_max[i]),
                "q_lower": float(model.lowerPositionLimit[i]),
                "q_upper": float(model.upperPositionLimit[i]),
            }

    out = {"urdf": args.urdf, "nv": int(nv), "mc_samples": int(args.mc), "joints": joints}
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Wrote {args.out}: {len(joints)} joints, {args.mc} MC samples")
    # quick console preview of the biggest movers
    rows = sorted(joints.items(), key=lambda kv: kv[1]["M_max"], reverse=True)
    print(f"\n{'joint':<22}{'M_neutral':>11}{'M_min':>10}{'M_median':>11}{'M_max':>10}")
    for name, d in rows[:14]:
        print(f"{name:<22}{d['M_neutral']:>11.5f}{d['M_min']:>10.5f}{d['M_median']:>11.5f}{d['M_max']:>10.5f}")


if __name__ == "__main__":
    main()
