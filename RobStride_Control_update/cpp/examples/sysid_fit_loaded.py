#!/usr/bin/env python3
"""
Gray-box system ID of a RobStride joint *under load* (pendulum link attached).

The link is modeled as a real body on a horizontal hinge with gravity ON, so the
pendulum dynamics live in the MODEL (no FFT pendulum-contamination, no gravity
feedforward needed). We:
  - FREEZE the rotor armature at the bare value (default 0.124, from the bare fit)
    to break the rotor-vs-link inertia degeneracy,
  - identify the LINK's full physical inertia (mass/CoM/tensor via pseudo-inertia
    -> CoM gives mgr and the gravity reference automatically),
  - identify joint damping + frictionloss.

Output: link inertia about the joint axis, link CoM (=> mgr), and the loaded
effective inertia  I_eff = armature + I_link_about_axis.

Run with the sysid venv:
  PY=.../sysid_venv/bin/python
  $PY sysid_fit_loaded.py <loaded_csv> [--armature 0.124] [--mass 2.8]
"""
import argparse, numpy as np, pandas as pd
import mujoco
from mujoco import sysid

MODEL_XML = """
<mujoco model="rs_loaded">
  <compiler angle="radian"/>
  <option integrator="implicitfast" timestep="0.001" gravity="0 0 -9.81">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="j" type="hinge" axis="0 1 0"
             armature="{arm}" damping="{damp}" frictionloss="{fric}"/>
      <inertial pos="{com} 0 0" mass="{mass}" diaginertia="{ix} {iyz} {iyz}"/>
    </body>
  </worldbody>
  <actuator><motor name="tau" joint="j"/></actuator>
  <sensor>
    <jointpos name="position" joint="j"/>
    <jointvel name="velocity" joint="j"/>
  </sensor>
</mujoco>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--armature", type=float, default=0.124, help="frozen bare rotor inertia")
    ap.add_argument("--mass", type=float, default=0.1)
    ap.add_argument("--length", type=float, default=1.0,
                    help="rod length (m) — sets the initial inertial guess (uniform rod)")
    ap.add_argument("--damp0", type=float, default=0.3)
    ap.add_argument("--fric0", type=float, default=0.4)
    ap.add_argument("--dt", type=float, default=0.001)
    ap.add_argument("--torque", choices=["actual", "commanded"], default="actual",
                    help="torque driving the model: actual_torque (physical) or torque_ff")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    def col(b): return next((c for c in df.columns if c.startswith(b)), None)
    t_ms = df["timestamp_ms"].to_numpy(float); t = (t_ms - t_ms[0]) / 1000.0
    tcol = "actual_torque_m" if args.torque == "actual" else "torque_ff_m"
    cmd = df[col(tcol)].to_numpy(float)
    pos = np.deg2rad(df[col("actual_deg_m")].to_numpy(float))
    vel = np.deg2rad(df[col("vel_deg_s_m")].to_numpy(float))
    tg = np.arange(0.0, t[-1], args.dt)
    cmd = np.interp(tg, t, cmd); pos = np.interp(tg, t, pos); vel = np.interp(tg, t, vel)
    print(f"[data] {args.csv}: {len(tg)} samp @ {1/args.dt:.0f}Hz, {tg[-1]:.1f}s | "
          f"cmd peak={np.abs(cmd).max():.2f}Nm  pos swing=±{np.rad2deg(pos-pos.mean()).__abs__().max():.0f}°")

    com = args.length / 2.0                                  # uniform-rod CoM
    iyz = max(1e-6, args.mass * args.length**2 / 12.0)       # rod inertia about CoM (perp)
    ix = max(1e-6, iyz * 0.01)                               # thin rod: tiny about its own axis
    spec = mujoco.MjSpec.from_string(MODEL_XML.format(
        arm=args.armature, damp=args.damp0, fric=args.fric0,
        mass=args.mass, com=com, ix=ix, iyz=iyz))
    model = spec.compile()

    initial_state = sysid.create_initial_state(
        model, qpos=np.array([pos[0]]), qvel=np.array([vel[0]]))
    control_ts = sysid.TimeSeries(tg, cmd.reshape(-1, 1))
    sensor_ts = sysid.TimeSeries.from_names(
        tg, np.column_stack([pos, vel]), model, names=["position", "velocity"])

    params = sysid.ParameterDict()
    # link full physical inertia (mass kept near known value)
    params.add(sysid.body_inertia_param(
        spec, model, "link", inertia_type=sysid.InertiaType.Pseudo,
        mass_bound_mult=np.array([0.85, 1.15]), param_name="link_inertia"))
    # joint friction (armature stays frozen in the XML)
    params.add(sysid.Parameter("damping", nominal=args.damp0,
        min_value=0.0, max_value=5.0,
        modifier=lambda s, p: setattr(s.joint("j"), "damping",
                                      np.array([p.value[0], 0.0, 0.0]))))
    params.add(sysid.Parameter("frictionloss", nominal=args.fric0,
        min_value=0.0, max_value=5.0,
        modifier=lambda s, p: setattr(s.joint("j"), "frictionloss", p.value[0])))

    ms = sysid.ModelSequences(
        "rs_loaded", spec, "multisine", initial_state, control_ts, sensor_ts)
    residual_fn = sysid.build_residual_fn(models_sequences=[ms])
    print(f"[fit] armature FROZEN at {args.armature}; identifying link inertia + friction ...")
    opt_params, opt_result = sysid.optimize(
        initial_params=params, residual_fn=residual_fn, optimizer="mujoco", verbose=True)

    # --- read out the identified link inertial from the optimized spec ---
    final_spec = sysid.apply_param_modifiers_spec(opt_params, spec)
    b = final_spec.body("link")
    m = float(b.mass); com = np.array(b.ipos); Idiag = np.array(b.inertia)
    r = np.linalg.norm(com[[0, 2]])               # CoM radius in the gravity plane (x,z)
    I_com_axis = Idiag[1]                          # inertia about the y (hinge) axis, CoM frame
    I_link_axis = I_com_axis + m * r**2           # parallel-axis to the joint
    I_eff = args.armature + I_link_axis
    mgr = m * 9.81 * r

    print("\n================ LOADED RESULT ================")
    print(f"  link mass        : {m:.3f} kg   (known ~{args.mass})")
    print(f"  link CoM radius  : {r*1000:.1f} mm  (=> mgr = {mgr:.2f} Nm)")
    print(f"  I_link about axis: {I_link_axis:.4f} kg·m^2  (CoM term m·r²={m*r**2:.4f} + I_com={I_com_axis:.4f})")
    print(f"  armature (frozen): {args.armature:.4f}")
    print(f"  --> I_eff (loaded): {I_eff:.4f} kg·m^2")
    print(f"  damping          : {opt_params['damping'].value[0]:.4f}")
    print(f"  frictionloss     : {opt_params['frictionloss'].value[0]:.4f}")
    print("===============================================")


if __name__ == "__main__":
    main()
