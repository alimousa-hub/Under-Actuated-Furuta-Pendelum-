# 🔄 Furuta Pendulum Balancing System — MATLAB/Simulink

A full modeling, simulation, and control design project for a **Furuta (Rotary Inverted) Pendulum** — a classic nonlinear unstable control systems challenge. The system is mathematically modeled using the Euler-Lagrange method, linearized around the upright equilibrium, and stabilized using both **PID** and **Full State Feedback (LQR-style)** controllers, all simulated in MATLAB/Simulink.

> 📚 *ELE 443 – Control Systems Lab | Lebanese American University | December 2025*
> 
> *Ali Moussa & Hussein Itani*

---

## 📁 Files

| File | Description |
|---|---|
| `MatlabFIle.m` | MATLAB script: system parameters, state-space derivation, pole placement, open-loop simulation |
| `Simulinkfile.slx` | Simulink model: plant simulation, PID controller, full state feedback controller |
| `project_report.docx` | Full technical report with derivations, results, and analysis |
| `ControlLabproject.pptx` | Project presentation slides |

---

## ⚙️ System Overview

The Furuta pendulum consists of a **rotating arm** driven by a motor, with a **freely swinging pendulum** attached at its end. The goal is to balance the pendulum in the upright vertical position by controlling the motor input voltage.

**System Variables:**
| Symbol | Description |
|---|---|
| α | Pendulum angle (from vertical upright) |
| θ | Arm rotation angle |
| Vm | Motor input voltage (control input) |
| mp | Pendulum mass = 0.024 kg |
| r | Arm length = 0.085 m |
| L | Pendulum length = 0.129 m |

---

## 📐 Mathematical Modeling

### 1. Euler-Lagrange Formulation
The equations of motion were derived using the Lagrangian method (T − V), yielding two coupled nonlinear differential equations for α and θ.

### 2. Linearization
The nonlinear model was linearized about **α = 0** (upright position) using small angle approximation:
- sin α ≈ α, cos α ≈ 1, sin²α ≈ 0

### 3. State-Space Representation
The linearized system is expressed as:

```
x = [α, α̇, θ, θ̇]ᵀ
```

With matrices A, B, C, D derived from the physical parameters and inertia constants:
```
Δ = (Jr + mp·r²)(Jp + mp·s²) − (½·mp·r·L)²
```

---

## 🎮 Control Design

### PID Controller (Two Independent Loops)

| Loop | Controller | Reason |
|---|---|---|
| Pendulum angle α | **PD** | Fast correction needed; integral would destabilize |
| Arm angle θ | **PI** | Precise positioning + eliminate steady-state error |

Both controllers were tuned using **Simulink's PID Tuner** on the linearized state-space model.

**Results:**
- ✅ α stabilized to upright position with good damping
- ✅ θ tracks reference with zero steady-state error

---

### Advanced Controller — Full State Feedback (Pole Placement)

Desired performance specs:
- Overshoot: **15%**
- Settling time: **2 seconds**

From these specs, the damping ratio ζ and natural frequency ωn were calculated, and 4 closed-loop poles were placed:

```matlab
p1, p2 = dominant complex conjugate poles
p3 = -3ωn  (fast real pole)
p4 = -5ωn  (fast real pole)
```

The state feedback gain matrix **K** was computed using MATLAB's `place()` function, and a feedforward gain **N** was added for reference tracking.

**Results:**
- ✅ Pendulum stabilized from a 15° initial disturbance
- ✅ Better performance than PID in transient response

---

## 🚀 How to Run

### MATLAB Script
1. Open `MatlabFIle.m` in MATLAB
2. Run the script — it will:
   - Compute system matrices A, B, C, D
   - Display open-loop eigenvalues
   - Plot open-loop step response
   - Compute pole placement gain matrix K

### Simulink Model
1. Open `Simulinkfile.slx` in MATLAB/Simulink
2. Make sure `MatlabFIle.m` has been run first (loads parameters into workspace)
3. Run the simulation
4. View results in the scope blocks

**Required:** MATLAB R2020a or later with Simulink and Control System Toolbox

---

## 📊 Key Results

- The **open-loop** system is unstable — the pendulum falls without control
- The **PID controller** successfully balances the pendulum with acceptable overshoot
- The **full state feedback controller** achieves faster settling with less oscillation
- Both controllers were validated in simulation before hardware testing

---

## 👨‍💻 Authors

**Ali Moussa** — Mechatronics Engineering Student @ Lebanese American University 🇱🇧  
**Hussein Itani** — Mechatronics Engineering Student @ Lebanese American University 🇱🇧
