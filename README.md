# State-Space Analysis of a Stretch3 Robot for Pick-and-Place

A proof of concept for **flexible automation**: instead of a fixed-base industrial arm
confined to a workcell, the **Stretch3 mobile manipulator** performs a pick-and-place task
in the **MuJoCo** physics simulator, while **MATLAB** derives and validates the system's
state-space representation.

The robot picks and places a cylinder and a cuboid, exercising 8 degrees of freedom (base
x/y, arm prismatic y/z, wrist roll/pitch/yaw, and the gripper). A reduced 2-DOF model
(arm lift and extension/retraction) is used for the control and state-space analysis.

## Modeling

Each joint is modeled as a mass–spring–damper, `m·q̈ + b·q̇ + k·q = τ`, with first-order
actuator dynamics `ȧ = −(1/T)·a + (1/T)·u`. The state and input vectors are:

```
x = [q1 q2 q̇1 q̇2 a1 a2]ᵀ        u = [u1 u2]ᵀ
```

Parameters: joint 1 `(m,b,k) = (1, 2, 10)`, joint 2 `(m,b,k) = (1, 1, 3)`, actuator time
constant `T = 0.1`. The state-space matrices are formed as `A = ∂f/∂x`, `B = ∂f/∂u`,
`C = I₆`, `D = 0`.

## Control

- **PD controller** — drives all 8 DOFs in MuJoCo from position error and joint velocity
  (`τ = Kp(q_ref − q) − Kd·q̇`), tuned directly in the robot's XML for fast motion with
  minimal overshoot and no sustained oscillation. Precise but comparatively slow.
- **LQR controller** — designed on the reduced 2-DOF model to improve responsiveness, with
  `Q` (state weighting) and `R` (control effort) tuned for speed. Faster settling and more
  aggressive control than PD, at the cost of a small undershoot on the lift DOF.

## Analysis (MATLAB)

The A, B, C, D matrices are built symbolically, then the system is checked for:

- **Controllability** — `ctrb(A,B)` is full rank (6) → completely controllable.
- **Observability** — `obsv(A,C)` is full rank (6) → completely observable.
- **Stability** — all eigenvalues of A have strictly negative real parts → internally
  asymptotically stable; being LTI and proper, also BIBO stable.

These results indicate the system is feasible to implement and test on real hardware.

## Repository contents

- MuJoCo simulation (Python) for the Stretch3 pick-and-place of the cuboid and cylinder.
- PD controller script and XML configuration.
- LQR design (`build_lqr_gain`) on the reduced 2-DOF model.
- MATLAB scripts for the state-space matrices and controllability/observability/stability analysis.
- `docs/` — the final report.

## Running

**Simulation (MuJoCo)** — install MuJoCo and Python dependencies, then run the simulation
script; the PD gains are set in the controller script / XML and tuned iteratively.

**Analysis (MATLAB)** — run the state-space script to build A, B, C, D and report the
controllability, observability, and stability results.

## Built on

[Stretch MuJoCo](https://github.com/hello-robot/stretch_mujoco) ·
[Stretch AI](https://github.com/hello-robot/stretch_ai) ·
[MuJoCo Menagerie — Stretch 3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/hello_robot_stretch_3)

## Course

MAE 506 — Arizona State University

## Authors

Abhinav Viswanathan · Arvind Kaushik · Reuben Kukar · Jatin Satyam
