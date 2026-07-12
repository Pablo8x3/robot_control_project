<div align="center">

# 🦾 Robot Control Project — 3-DOF Manipulator

**Kinematic & dynamic modeling + design and benchmarking of 5 control strategies in MATLAB/Simulink**

[![MATLAB](https://img.shields.io/badge/MATLAB-Symbolic%20%2B%20Control-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-Model--Based-blue?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Robotics Toolbox](https://img.shields.io/badge/Robotics%20Toolbox-Peter%20Corke-green)](https://petercorke.com/toolboxes/robotics-toolbox/)
[![Status](https://img.shields.io/badge/status-completed-success)]()

<img src="docs/figures/robot_3dof.png" width="55%" alt="3-DOF robot rendered with the Robotics Toolbox">

*From symbolic dynamics to a full controller benchmark — every controller stress-tested against **±20 % parametric uncertainty**.*

</div>

---

## 📌 Overview

This project develops the **complete modeling and control pipeline of a 3-DOF articulated manipulator**, from first principles to closed-loop simulation:

1. **Kinematics** — Denavit–Hartenberg parametrization, homogeneous transforms, analytical Jacobian and **singularity analysis**.
2. **Dynamics** — symbolic derivation of the full model `τ = M(q)q̈ + C(q,q̇)q̇ + G(q)` via **two independent methods** (Euler–Lagrange and recursive Newton–Euler) used to cross-validate each other.
3. **Trajectory generation** — a 3-D circular path solved through inverse kinematics.
4. **Control** — five controllers designed, simulated in Simulink and compared under identical conditions: **decentralized PD, decentralized PID, nested (cascade) control, model-based feed-forward, and computed torque**.
5. **Robustness** — every controller is re-evaluated with **20 % parametric uncertainty** injected into the plant to expose which designs actually survive model mismatch.

Developed for the *Robot Programming and Control* course — B.Eng. in Robotics and Mechatronics, Universidad Loyola Andalucía.

---

## 🗂️ Table of Contents

- [Robot Model](#-robot-model)
- [Kinematics & Trajectory](#-kinematics--trajectory)
- [Dynamic Model](#-dynamic-model)
- [Controller Design](#-controller-design)
  - [Sample Time Selection](#sample-time-selection)
  - [Decentralized PD](#1-decentralized-pd)
  - [Decentralized PID](#2-decentralized-pid)
  - [Nested (Cascade) Control](#3-nested-cascade-control)
  - [Feed-Forward Control](#4-model-based-feed-forward)
  - [Computed Torque](#5-computed-torque)
- [📊 Results & Comparison](#-results--comparison)
- [Repository Structure](#-repository-structure)
- [Getting Started](#-getting-started)

---

## 🤖 Robot Model

The manipulator is a **3-DOF anthropomorphic arm** (all revolute joints) with an offset elbow geometry, modeled with Peter Corke's Robotics Toolbox (`Link` / `SerialLink`).

| Parameter | Link 1 | Link 2 | Link 3 / 4 |
|---|---|---|---|
| Length [m] | 0.54 | 0.50 | 0.45 / 0.40 |
| Mass [kg] | 4.0 | 2.7 | 2.0 |
| Motor inertia J<sub>m</sub> [kg·m²] | 0.15 | 0.25 | 0.08 |
| Gear ratio | 25 | 20 | 25 |
| Viscous friction B<sub>m</sub> | 0.0014 | 0.0019 | 0.0022 |

The DH frames were derived by hand before being translated into code — the geometry includes a non-trivial offset (`atan2(L2, L3)`) that couples two link lengths into a single equivalent link:

<div align="center">
<img src="docs/figures/robot_sketch.jpg" width="42%" alt="Hand-drawn robot schematic">
<img src="docs/figures/dh_axes.jpg" width="42%" alt="DH axes assignment">
</div>

---

## 📐 Kinematics & Trajectory

The analytical Jacobian was derived symbolically and used to locate the **singular configurations** of the arm, where `det(J) = 0` and the manipulator instantaneously loses a degree of freedom:

- `q₃ = kπ` (elbow fully extended/folded)
- `q₂` reaching the arm-alignment condition `arctan(L₃/L₂)`
- `q₂ + q₃` reaching that same alignment condition

To keep the closed-loop experiments well-conditioned, all reference trajectories are constrained to `q₁ ∈ [0, π/2]`, `q₂, q₃ ∈ [0, π/8]` — safely away from the singularity loci.

As a kinematic validation task, the end-effector traces a **circle of radius 0.85 m** centered at `[0, 0, 1.04] m`, solved via inverse kinematics (`ikine` with a position-only mask):

<div align="center">
<img src="docs/figures/circle_trajectory.png" width="46%" alt="Circular trajectory traced by the end effector">
<img src="docs/figures/circle_joints.png" width="46%" alt="Joint trajectories during the circle">
</div>

---

## ⚙️ Dynamic Model

The full rigid-body dynamics

$$\tau = M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + G(q)$$

were derived **symbolically twice**, with two independent formulations that must (and do) agree:

| Script | Method | Output |
|---|---|---|
| `Lagrangian_Method_3DOF_normal.m` | Euler–Lagrange (pseudo-inertia matrices) | Symbolic `M`, `C`, `G` |
| `NE_Method_3DOF.m` | Recursive Newton–Euler (outward/inward pass) | Joint torques `τ` |

Cross-deriving the model with both methods catches algebra mistakes that a single derivation would silently propagate into every controller downstream.

---

## 🎛️ Controller Design

### Sample Time Selection

Before tuning anything, the discrete sample time `Ts` is chosen with **three independent criteria** — and the most conservative one wins:

1. **Bandwidth criterion** — `Ts = 2π / (10·max(ωₙ))` (Nyquist–Shannon with 10× margin)
2. **Time-constant criterion** — `Ts = min(τᵢ)/20`, with `τᵢ = Jₘᵢ/Bₘᵢ`
3. **Settling-time criterion** — ≥100 samples inside the 2 % settling window

All five controllers run on the resulting `Ts`, so the comparison is apples-to-apples.

---

### 1. Decentralized PD

Independent joint PD, tuned for **steady-state ramp error < 1 %**.

<div align="center">
<img src="docs/figures/pd_tracking.png" width="46%" alt="PD reference tracking">
<img src="docs/figures/pd_error.png" width="46%" alt="PD absolute error">
</div>

**Result:** max error ≈ 5×10⁻³ rad (< 0.3°). Trapezoidal error profile — no integral action, so the error plateaus mid-trajectory. Some overshoot in the control signal.

<details>
<summary>📉 <b>Robustness: +20 % parametric uncertainty</b></summary>
<div align="center">
<img src="docs/figures/pd_tracking_unc.png" width="46%" alt="PD tracking with uncertainty">
<img src="docs/figures/pd_error_unc.png" width="46%" alt="PD error with uncertainty">
</div>
Error stays in the same order of magnitude, but is no longer driven down as effectively — the price of having no integrator.
</details>

---

### 2. Decentralized PID

Designed **in the frequency domain**: the loop is shaped at a chosen crossover frequency (`ω = 50 rad/s`) with a **phase-margin target > 80°** per joint, verified with Bode/`margin` analysis.

<div align="center">
<img src="docs/figures/pid_tracking.png" width="46%" alt="PID reference tracking">
<img src="docs/figures/pid_error.png" width="46%" alt="PID absolute error">
</div>

**Result:** max error ≈ 1×10⁻³ rad. The integral action changes the error shape from trapezoidal to a smooth first-order decay, and reduces the control overshoot seen in the PD case — less mechanical stress on the actuators.

<details>
<summary>📉 <b>Robustness: +20 % parametric uncertainty</b></summary>
<div align="center">
<img src="docs/figures/pid_tracking_unc.png" width="46%" alt="PID tracking with uncertainty">
<img src="docs/figures/pid_error_unc.png" width="46%" alt="PID error with uncertainty">
</div>
Slight overshoot appears in the error, but the integrator absorbs the steady-state component — the degradation vs. nominal is minimal.
</details>

---

### 3. Nested (Cascade) Control

Inner velocity loop + outer position loop per joint.

<div align="center">
<img src="docs/figures/nested_tracking.png" width="46%" alt="Nested control reference tracking">
<img src="docs/figures/nested_control.png" width="46%" alt="Nested control actions">
</div>

**Result:** accuracy comparable to PD/PID, but the **control-action overshoot is practically eliminated** — the smoothest input signals of all the feedback-only schemes. Under uncertainty, joint 1 (the one with the largest travel and no singularity-imposed bound) deviates up to ≈ 0.01 rad.

<details>
<summary>📉 <b>Robustness: +20 % parametric uncertainty</b></summary>
<div align="center">
<img src="docs/figures/nested_tracking_unc.png" width="46%" alt="Nested tracking with uncertainty">
<img src="docs/figures/nested_error_unc.png" width="46%" alt="Nested error with uncertainty">
</div>
</details>

---

### 4. Model-Based Feed-Forward

The inverse dynamic model feeds the nominal torque/voltage directly — **no feedback correction**.

<div align="center">
<img src="docs/figures/ff_tracking.png" width="46%" alt="Feed-forward reference tracking">
<img src="docs/figures/ff_error.png" width="46%" alt="Feed-forward error">
</div>

**Result:** the weakest tracker — max error ≈ 0.02 rad (≈ 1.15°) on joint 1. Control signals are large and smooth (near-sinusoidal), but every model mismatch and disturbance translates *directly* into tracking error. A textbook demonstration of why pure feed-forward needs a feedback companion.

<details>
<summary>📉 <b>Robustness: +20 % parametric uncertainty</b></summary>
<div align="center">
<img src="docs/figures/ff_tracking_unc.png" width="46%" alt="FF tracking with uncertainty">
<img src="docs/figures/ff_error_unc.png" width="46%" alt="FF error with uncertainty">
</div>
Disturbances are visible even in the raw tracking plots — no mechanism exists to reject them.
</details>

---

### 5. Computed Torque

Full feedback linearization: the dynamic model cancels the nonlinearities online, leaving a linear error system.

<div align="center">
<img src="docs/figures/ct_tracking.png" width="46%" alt="Computed torque reference tracking">
<img src="docs/figures/ct_error.png" width="46%" alt="Computed torque error">
</div>

**Result:** 🏆 the best of the benchmark — max error ≈ **7×10⁻⁵ rad nominal** and **< 4×10⁻³ rad under uncertainty**, with smooth control signals. The cost: it is the most computationally demanding scheme and the most dependent on model accuracy.

<details>
<summary>📉 <b>Robustness: +20 % parametric uncertainty</b></summary>
<div align="center">
<img src="docs/figures/ct_tracking_unc.png" width="46%" alt="CT tracking with uncertainty">
<img src="docs/figures/ct_error_unc.png" width="46%" alt="CT error with uncertainty">
</div>
</details>

---

## 📊 Results & Comparison

| Controller | Model-based | Max error (nominal) | Max error (+20 % unc.) | Control effort | Verdict |
|---|:---:|:---:|:---:|---|---|
| **PD** | ❌ | ~5×10⁻³ rad | same order, slower decay | Some overshoot | Simple, solid baseline |
| **PID** | ❌ | ~1×10⁻³ rad | Minor degradation | Smooth | Best simplicity/precision trade-off |
| **Nested** | ❌ | ~PD-level | ≤ 1×10⁻² rad (joint 1) | Smoothest (no overshoot) | Kindest to the actuators |
| **Feed-forward** | ✅ | ~2×10⁻² rad | Visibly degraded | Large, smooth | Don't use alone |
| **Computed torque** | ✅ | **~7×10⁻⁵ rad** | **< 4×10⁻³ rad** | Smooth | 🏆 Best accuracy — if you can afford the model |

**Takeaway:** with an accurate model and enough compute, **computed torque wins by two orders of magnitude**. When model confidence or computational budget is limited, **PID / nested control** deliver 90 % of the performance at a fraction of the complexity.

---

## 📁 Repository Structure

```
robot_control_project/
├── README.md
├── docs/
│   └── figures/                      # Plots used in this README
│
│   # ── Modeling ──────────────────────────────────────────
├── circle_drawing.m                  # Kinematic model + circular trajectory (ikine)
├── Jacobiano.m                       # Symbolic Jacobian & forward kinematics
├── Lagrangian_Method_3DOF_normal.m   # Symbolic dynamics via Euler–Lagrange
├── NE_Method_3DOF.m                  # Symbolic dynamics via Newton–Euler
├── sinusoidal_torque_inputs.m        # Open-loop excitation test
│
│   # ── Controller design & simulation drivers ───────────
├── controlled_robot_def_PD.m         # PD design + Ts selection + sim + plots
├── controlled_robot_def_PID.m        # PID frequency-domain design + sim + plots
├── controlled_robot_def_Nested.m     # Cascade control + sim + plots
├── controlled_robot_def_freedfwrd.m  # Feed-forward + sim + plots
├── controlled_robot_def_ctorque.m    # Computed torque + sim + plots
│
│   # ── Simulink models ──────────────────────────────────
├── Open_loop_robot.slx
├── cl_PD_bueno.slx
├── cl_PID.slx
├── cl_Nested.slx
├── cl_fforward.slx
├── cl_ctorque.slx
├── closed_loop_independent_control_PD.slx
│
│   # ── Simulation results ───────────────────────────────
├── results_PD/
└── results_Nested/
```

---

## 🚀 Getting Started

**Requirements**

- MATLAB + Simulink
- [Robotics Toolbox for MATLAB (Peter Corke, RTB 10.x)](https://petercorke.com/toolboxes/robotics-toolbox/) — the classic `SerialLink` API

**Run a controller benchmark**

```matlab
% Each controller has a self-contained driver script:
%   1. Builds the robot object and computes Ts
%   2. Opens & runs the corresponding Simulink model
%   3. Plots tracking, error and control-action figures

run('controlled_robot_def_ctorque.m')   % e.g., computed torque
```

To reproduce the robustness study, each Simulink model contains a *perturbed plant* branch where link masses/lengths are scaled by the uncertainty factor `unc = 0.2`.

**Reproduce the trajectory**

```matlab
run('circle_drawing.m')   % 3-D circle + joint-space solution via ikine
```

---

## 👤 Author

**Pablo Carneado López** — B.Eng. Robotics and Mechatronics, Universidad Loyola Andalucía
Course: *Robot Programming and Control* · Supervised by Prof. Guillermo Bejarano Pellicer

📄 The full technical report (kinematic derivations, dynamic matrices, controller tuning rationale) is available in [`docs/Robot_Control_Project.pdf`](docs/Robot_Control_Project.pdf).
