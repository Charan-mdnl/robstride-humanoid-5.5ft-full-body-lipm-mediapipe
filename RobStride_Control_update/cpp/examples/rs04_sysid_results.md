# RobStride RS04 — Gray-Box System Identification (results + method)

Worked results for identifying the RS04 actuator parameters with **MuJoCo's
sysid toolbox** (gray-box, time-domain), following the observie / kevin_zakka
method documented in `docs/learning/02_real_to_sim_rs02_sysid.md`. This
supersedes the FFT/Bode estimates in `bode.md` (see "Why gray-box" below).

---

## TL;DR results (actual-torque basis)

| Quantity | Value | Notes |
|---|---|---|
| **Bare armature** (reflected rotor inertia, our "I") | **0.037 kg·m²** | repeatable across 3 runs (0.036 / 0.0385 / 0.0357) |
| **Total friction** | **≈ 0.78 N·m at 5 rad/s** | well-determined |
| viscous damping | ~0.05–0.11 N·m·s/rad | split is run-dependent (see caveat) |
| Coulomb frictionloss | ~0.22–0.51 N·m | split is run-dependent |
| **Loaded I_eff** (rotor + 2.8 kg / 0.3 m link) | **0.090 kg·m²** | rotor 0.037 + link 0.053 |
| link inertia about axis | 0.053 kg·m² | CoM ~129 mm, mgr ~3.5–4 N·m |

All values are on the **actual-torque basis** (see the torque-delivery finding).

---

## Two critical findings

### 1. Use ACTUAL torque, not commanded
The motor delivers/reports only **~46% of the commanded torque**
(`actual_torque ≈ 0.46 × torque_ff`, repeatable across runs). Because
`I = τ / α`, feeding the *commanded* torque inflates every inertia estimate by
~1/0.46 ≈ 2.2×:

| | commanded basis | actual basis |
|---|---|---|
| bare armature | 0.124 | **0.037** |
| loaded I_eff | 0.235 | **0.090** |

→ Always fit with `--torque actual`. The rotor responds to the *physical*
torque, best measured by `actual_torque` (current-based). The fit also fails
outright with commanded torque at high amplitude (model over-accelerates).

> Absolute caveat: this still trusts `actual_torque`'s own calibration. The only
> absolute anchor is **gravity** (static-hold `mgr`). Do a static gravity-hold to
> confirm the torque scale for certain.

### 2. The FFT/Bode method under-reads inertia (~3×)
The old FFT estimate gave bare I ≈ 0.03 and loaded ≈ 0.08. The gray-box fit
gives 0.037 and 0.090 — *broadly consistent on the actual basis*, but the FFT's
single-`1/(Is+B)` model **ignores Coulomb friction** and is noise-limited at high
frequency, so its split and absolute values are unreliable. Gray-box separates
**armature / viscous damping / Coulomb frictionloss** simultaneously and handles
the loaded pendulum in the model (no FFT pendulum-contamination).

---

## Excitation: the observie multi-sine

`multi_motor_functionality.cpp` (torque mode `t`) now generates observie's
literal multi-sine:

```
torque(t) = amp × ( 1.0·sin(2π·1.0·t) + 0.6·sin(2π·3.4·t) + 0.3·sin(2π·7.4·t) )
```
- base_freq 1.0 Hz, ratios {1.0, 3.4, 7.4}, weights {1.0, 0.6, 0.3}
- **`amp` is the CLI amplitude arg (observie's leading coefficient), NOT the peak.**
  Peak ≈ `amp × (1.0+0.6+0.3) = 1.9 × amp`. So `amp = 3` → ±5.7 Nm commanded.
- No clamping → peaks are never clipped.
- Verified spectrum: clean tones at 1.0 / 3.4 / 7.4 Hz, magnitudes 1.0 / 0.54 / 0.20.

**Recommended bare run:** no load, `kp=0 kd=0`, 1000 Hz, ~10 s, `amp ≈ 2.6`
(→ ±5 Nm commanded, ~±2.3 Nm actual). Keep amplitude low so the motor stays in
the linear regime (high amplitude → 800+ deg/s → torque-speed-curve droop →
the constant-torque model breaks and the fit fails).

---

## How to run the fits

Environment (mujoco[sysid], separate venv to avoid the pinocchio/numpy clash):
```
PY=/home/xp/Desktop/RobStride_Control_update/RobStride_Control_update/sysid_venv/bin/python
```

Bare:
```
$PY ../examples/sysid_fit.py <bare_csv> --torque actual
# fits: armature, damping, frictionloss
```

Loaded (link on, gravity in the model; armature frozen at the bare value):
```
$PY ../examples/sysid_fit_loaded.py <loaded_csv> --torque actual --armature 0.037 --mass 2.8
# fits: link full inertia (CoM => mgr) + damping + frictionloss
# reports I_eff = armature + I_link_about_axis
```

A good fit shows residual `y < ~0.1` and parameters NOT pinned at their bounds.

---

## Open items / how to tighten

1. **Friction split (viscous vs Coulomb)** — ambiguous from one broadband run
   (only the *total* is constrained). Fix: run one slow/low-amp and one fast/
   higher-amp test and fit them **jointly** with shared parameters; the speed
   contrast separates Coulomb (constant) from viscous (∝ velocity).
2. **Absolute torque scale** — anchor with a **static gravity-hold** (`mgr`) to
   validate `actual_torque`'s calibration.
3. **Fresh loaded run** — the loaded fit above used the *old-excitation* run
   `154142` (residual 0.57). Redo with the observie excitation (link on,
   `amp ≈ 2.6`) for a clean loaded fit like the bare runs (residual ~0.09).
4. **Link mass pins at the upper bound** in the loaded fit (3.22 vs 2.8 kg) —
   trust `I_eff`, not the mass/CoM split, until the gravity anchor is done.

---

## Reference CSVs

| CSV | role | result |
|---|---|---|
| `motor_log_aligned_20260611_141647.csv` | bare (old excitation) | armature 0.036 |
| `motor_log_aligned_20260616_185219.csv` | bare (observie) | armature 0.0385, residual 0.10 |
| `motor_log_aligned_20260616_185806.csv` | bare (observie) | armature 0.0357, residual 0.085 |
| `motor_log_aligned_20260611_154142.csv` | loaded (old excitation) | I_eff 0.090 |

Scripts: `sysid_fit.py`, `sysid_fit_loaded.py` (this dir). Model: minimal
single-hinge + torque motor; armature carries the reflected rotor inertia.
