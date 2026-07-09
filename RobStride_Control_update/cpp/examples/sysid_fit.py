#!/usr/bin/env python3
"""
Gray-box system ID of a RobStride joint using MuJoCo's sysid toolbox.

Replays the recorded *commanded* torque (torque_ff) through a minimal
one-hinge MuJoCo model and fits:
    armature      - reflected rotor inertia through the gearbox  (== our "I")
    damping       - viscous friction (== our "B")
    frictionloss  - Coulomb (constant) friction  (we previously ignored this)
so that simulated joint pos/vel match the measured pos/vel.

This is the @observie / kevin_zakka method (docs/learning/02_real_to_sim).

Run with the dedicated sysid venv:
  PY=/home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/sysid_venv/bin/python
  $PY sysid_fit.py <aligned_csv> [--arm0 0.03] [--damp0 0.25] [--fric0 0.1]
"""
import argparse, numpy as np, pandas as pd
import mujoco
from mujoco import sysid

MODEL_XML = """
<mujoco model="rs_joint">
  <compiler angle="radian"/>
  <option integrator="implicitfast" timestep="0.001" gravity="0 0 0">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="rotor">
      <inertial pos="0 0 0" mass="0.001" diaginertia="1e-8 1e-8 1e-8"/>
      <joint name="j" type="hinge" axis="0 0 1"
             armature="{arm}" damping="{damp}" frictionloss="{fric}"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="tau" joint="j"/>
  </actuator>
  <sensor>
    <jointpos name="position" joint="j"/>
    <jointvel name="velocity" joint="j"/>
  </sensor>
</mujoco>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--arm0", type=float, default=0.03)
    ap.add_argument("--damp0", type=float, default=0.25)
    ap.add_argument("--fric0", type=float, default=0.1)
    ap.add_argument("--dt", type=float, default=0.001)
    ap.add_argument("--torque", choices=["actual", "commanded"], default="actual",
                    help="which torque drives the model: actual_torque (physical) "
                         "or torque_ff (commanded). Use 'actual' when the motor "
                         "does not deliver the full command.")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    def col(b): return next((c for c in df.columns if c.startswith(b)), None)
    t_ms = df["timestamp_ms"].to_numpy(float)
    t = (t_ms - t_ms[0]) / 1000.0
    tcol = "actual_torque_m" if args.torque == "actual" else "torque_ff_m"
    cmd = df[col(tcol)].to_numpy(float)                  # torque driving the rotor
    pos = np.deg2rad(df[col("actual_deg_m")].to_numpy(float))
    vel = np.deg2rad(df[col("vel_deg_s_m")].to_numpy(float))

    # resample onto a uniform dt grid (MuJoCo needs fixed timestep)
    tg = np.arange(0.0, t[-1], args.dt)
    cmd = np.interp(tg, t, cmd)
    pos = np.interp(tg, t, pos)
    vel = np.interp(tg, t, vel)
    print(f"[data] {args.csv}: {len(tg)} samples @ {1/args.dt:.0f} Hz, "
          f"{tg[-1]:.1f}s | cmd peak={np.abs(cmd).max():.2f} Nm")

    # build model spec at nominal params
    spec = mujoco.MjSpec.from_string(
        MODEL_XML.format(arm=args.arm0, damp=args.damp0, fric=args.fric0))
    model = spec.compile()

    initial_state = sysid.create_initial_state(
        model, qpos=np.array([pos[0]]), qvel=np.array([vel[0]]))
    control_ts = sysid.TimeSeries(tg, cmd.reshape(-1, 1))
    sensor_ts = sysid.TimeSeries.from_names(
        tg, np.column_stack([pos, vel]), model, names=["position", "velocity"])

    # parameters to identify
    params = sysid.ParameterDict()
    params.add(sysid.Parameter("armature", nominal=args.arm0,
        min_value=0.001, max_value=0.5,
        modifier=lambda s, p: setattr(s.joint("j"), "armature", p.value[0])))
    params.add(sysid.Parameter("damping", nominal=args.damp0,
        min_value=0.0, max_value=5.0,
        modifier=lambda s, p: setattr(s.joint("j"), "damping",
                                      np.array([p.value[0], 0.0, 0.0]))))
    params.add(sysid.Parameter("frictionloss", nominal=args.fric0,
        min_value=0.0, max_value=5.0,
        modifier=lambda s, p: setattr(s.joint("j"), "frictionloss", p.value[0])))

    ms = sysid.ModelSequences(
        "rs_joint", spec, "multisine", initial_state, control_ts, sensor_ts)
    residual_fn = sysid.build_residual_fn(models_sequences=[ms])

    print("[fit] optimizing armature / damping / frictionloss ...")
    opt_params, opt_result = sysid.optimize(
        initial_params=params, residual_fn=residual_fn,
        optimizer="mujoco", verbose=True)

    print("\n================ RESULT ================")
    for k in ["armature", "damping", "frictionloss"]:
        print(f"  {k:<13} {params[k].value[0]:.5f}  ->  {opt_params[k].value[0]:.5f}")
    try:
        print(f"  final cost: {opt_result.cost:.4e}")
    except Exception:
        pass
    print("========================================")


if __name__ == "__main__":
    main()
