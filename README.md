# 3-DOF RRR Robot Parameter Identification & Model-Based Control (Noisy Data Experiment)

An end-to-end framework for parameter estimation of a 3-DOF RRR serial manipulator operating under noisy sensor conditions. This repository features a custom symbolic regressor engine, a noise-isolation filtering pipeline, and a closed-loop Computed Torque Controller (CTC) validated via a Simscape Multibody digital twin.

Developed by **Abdelrahman Elzemrany**.

[📥 Click here to download a 170-second simulation video explaining the complete process.](A%20video.mp4)

---

## 📝 Project Overview

Conventional model-free controllers (like standalone PD loops) suffer from severe tracking degradation under dynamic loads. Unmodeled gravitational forces pull robotic link masses downward, causing permanent steady-state position sag. Reactive controllers require tracking errors to generate corrective torques, making high-precision path tracking impossible. 

A model-based control strategy (like Computed Torque Control) resolves this problem by canceling out the robot's physical weight and inertia equations on the fly. However, its tracking accuracy depends strictly on the precision of the underlying system parameters. 

Because direct physical measurements are unavailable and theoretical CAD baselines ignore structural variations caused by age and wear, an accurate model requires data-driven system identification. This project establishes an end-to-end parameter estimation pipeline to fit physical dynamic variables while resolving sensor noise and ensuring strict real-world physical realism.

---

## ⚙️ Mathematical & Algorithmic Breakdown

### 1. Custom Kinematic & Regressor Model (Step_1)
The framework utilizes proximal Denavit-Hartenberg (DH) convention parameters to systematically construct the system kinematics. Every physical robotic link maps to a 12-parameter dynamic vector, spanning:
* **10 Standard Inertial Parameters**: Mass ($m$), 3 Center of Mass components ($mx, my, mz$), and 6 unique elements of the inertia tensor ($I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{yz}, I_{xz}$).
* **2 Friction Parameters**: Viscous and Coulomb friction additions (`ViscousCoulomb`).

These variables are analytically decoupled from the nonlinear equations of motion to form a custom symbolic identification regressor matrix ($Y_b$). This structural rearrangement ensures that the multi-joint dynamic torque matches a linear-in-the-parameters equation model:

$$ \tau = Y_b(q, \dot{q}, \ddot{q}) \cdot \theta $$

### 2. Bounded Excitation Trajectory Optimizer (Step_2)
To guarantee optimal noise rejection during estimation, the robot must follow a trajectory that excites all parameters evenly. The path is modeled as a multi-harmonic finite Fourier series. 

Standard optimization algorithms manipulate 11 free parameters per joint, which frequently triggers harsh startup transients. To eliminate this issue, this pipeline implements an analytical variable reduction down to 8 variables per joint. By analytically resolving the fundamental terms ($q_0, a_1, b_1$), the trajectory guarantees zero-locked boundary initial states:

$$ q(0) = 0, \quad \dot{q}(0) = 0, \quad \ddot{q}(0) = 0 $$

Using Sequential Quadratic Programming (`fmincon` with SQP), the optimizer sweeps through joint position, velocity, and acceleration limits to maximize parameter visibility by minimizing the condition number of the tracking observation matrix.

### 3. Simscape Plant Excitation Experiment (Step_3)
The optimized finite-harmonic reference signals are dispatched directly to the closed-loop tracking architecture. A continuous-time feedback controller drives the 3-DOF RRR Simscape digital twin across the workspace paths. During this phase, physical dynamic parameters, Viscous-Coulomb joint dampening forces, and high-frequency sensor measurement noise are generated concurrently within the Simscape plant model.

### 4. Data Extraction, Decimation & Windowing (Step_4)
Raw sensor captures output enormous high-frequency time-series datasets that strain processor memory allocations. Step_4 executes a downsampling data decimation step to lower the overall sample density while perfectly preserving the underlying low-frequency rigid-body dynamics. 

Simultaneously, a localized windowing array crops out the first and last chunks of data (e.g., a 100-sample window baseline). This eliminates aggressive transient edge spikes induced by numerical central-difference gradients at the absolute boundaries of the data collection window.

### 5. High-Fidelity Data Filtering & Conditioning (Step_5)
Unlike noise-free variants, parameter identification scripts fail when processing raw, noisy sensor data. Step_5 routes the downsampled position arrays through a zero-phase, 4th-order low-pass Butterworth filter architecture (`filtfilt`). Forward-backward filtering eliminates time-lag delays, preserving exact synchronization between joint variables. 

```text
               +----------------------------------+

               |        Raw Position Data [q]     |
               +----------------------------------+
                                |
                                v
               +----------------------------------+

               |  Zero-Phase Butterworth Filter   |
               +----------------------------------+
                                |
                                v
               +----------------------------------+

               |    Central Difference Method     |
               +----------------------------------+
                                |
                                v
               +----------------------------------+

               |       Joint Velocity [q_dot]     |
               +----------------------------------+
                                |
                                v
               +----------------------------------+

               |     Savitzky-Golay Smoothing     |
               +----------------------------------+
                                |
                                v
               +----------------------------------+

               |    Filtered Acceleration [q_ddot]|
               +----------------------------------+
```

Velocities ($\dot{q}$) and accelerations ($\ddot{q}$) are analytically synthesized via central-difference gradients, followed by a secondary low-pass filtering loop on the target torque channels ($\tau$) to isolate unmodeled sensor chatter before running the convex optimization engine.

### 6. Physical Consistency & Epsilon ($\varepsilon$) Sensitivity Analysis
To keep parameters physically realistic, the `fmincon` objective function evaluates tracking error alongside strict structural constraints. It evaluates eigenvalues over a $5 \times 5 \times 5$ configuration grid (125 distinct workspace poses) to enforce a strictly positive-definite Mass/Inertia matrix ($M(q) > 0$).

The user-tunable boundary threshold value `epsilon_val` acts as a crucial sensitivity parameter:
* **High Epsilon Threshold ($\varepsilon \ge 0.05$)**: Strongly guarantees physical conservatism and numerical stability, preventing matrix division spikes during high-gain control. However, setting it too high can over-constrain the solver, causing poor data-fitting accuracy.
* **Low Epsilon Threshold ($\varepsilon \le 0.01$)**: Minimizes torque reconstruction errors by giving the solver more freedom, but risks approaching structural singularities where the mass matrix loses its positive-definiteness ($M(q) \to 0$).

This repository leverages an optimized balance of `epsilon_val = 0.037`, protecting model-based controllers from inverse-matrix spikes under feedback noise.

### 7. Robust Computed Torque Control Subsystem (Step_11)
The identified dynamic parameter matrix components directly feed the model-based controller. The system processes feedforward nonlinear rigid-body forces under high tracking feedback noise according to the following control law formulation:

$$ \tau = M(q)\left[\ddot{q}_d + K_p e + K_d \dot{e}\right] + C(q, \dot{q})\dot{q} + G(q) + F(\dot{q}) $$

Where $e = q_d - q$ represents the localized tracking joint error vector loop array driving system errors to zero.

```text
q_d, q_d_dot, q_d_ddot 
      |
      v
[Tracking Error Loop] ---> u ---> [ Inverse Dynamics Model ] ---> Joint Torques ---> [ Simscape Robot ]
      ^                               (Using Identified \theta_b)                               |

      |                                                                                       |
      +---------------------------------- Feedback (q, q_dot) --------------------------------+
```

---

## 📂 Repository Architecture & Pipeline

Run the scripts in numerical order to move from kinematic setup to validated CTC control:

```text
robot-identification-repo/
├── Step_1_TheRegressorModel.m              <-- Custom symbolic matrix processing engine
├── Step_2_excitation_trajectory.m          <-- Fourier trajectory path optimizer
├── Step_3_ParameterExcitation.slx          <-- Data acquisition plant (Simscape Multibody)
├── Step_4_DataextractForParameterEstimation.slx <-- Raw data decimation, windowing, and logging
├── Step_5_Filtered_Parameter_estimation.m  <-- Data filtering and grid-constrained parameter solver
├── Step_6_testingTheEstimatedParameters.slx <-- Validation trajectory plant
├── Step_7_DataextractForSecondExpValidation.slx <-- Independent validation dataset parser
├── Step_8_Validation_of_Estimation.m       <-- Multitarget tracking verification tool
├── Step_9_ReformTheEstimatedMatrices.m     <-- Matrix decoupling reformer
├── Step_10_CheckPositive.m                 <-- 27,000-point physical QA guard
├── Step_11_Results.m                       <-- Metrics reporting and plotting utility
