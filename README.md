# 3-DOF RRR Robot Parameter Identification & Model-Based Control (Noisy Data Experiment)

An end-to-end framework for parameter estimation of a 3-DOF RRR serial manipulator operating under noisy sensor conditions. This repository features a custom symbolic regressor engine, a noise-isolation filtering pipeline, and a closed-loop Computed Torque Controller (CTC) validated via a Simscape Multibody digital twin.



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

$$ \tau = Y_b(q, \dot{q}, \ddot{q}) \cdot \theta_b $$

### 2. Bounded Excitation Trajectory Optimizer (Step_2)
To guarantee optimal noise rejection during estimation, the robot must follow a trajectory that excites all parameters evenly. The path is modeled as a multi-harmonic finite Fourier series. 

Standard optimization algorithms manipulate 11 free parameters per joint, which frequently triggers harsh startup transients. To eliminate this issue, this pipeline implements an analytical variable reduction down to 8 variables per joint. By analytically resolving the fundamental terms ($q_0, a_1, b_1$), the trajectory guarantees zero-locked boundary initial states:

$$ q(0) = 0, \quad \dot{q}(0) = 0, \quad \ddot{q}(0) = 0 $$

Using Sequential Quadratic Programming (`fmincon` with SQP), the optimizer sweeps through joint position, velocity, and acceleration limits to maximize parameter visibility by minimizing the condition number of the tracking observation matrix.

--------
<img width="1917" height="925" alt="image" src="https://github.com/user-attachments/assets/2c5949ac-72b7-40b4-bb17-b017f4b60291" />

Figure 1 The excitaion trajectory .

----------------

### 3. Simscape Plant Excitation Experiment (Step_3)
The optimized finite-harmonic reference signals are dispatched directly to the closed-loop tracking architecture. A continuous-time feedback controller drives the 3-DOF RRR Simscape digital twin across the workspace paths. During this phase, physical dynamic parameters, Viscous-Coulomb joint dampening forces, and high-frequency sensor measurement noise are generated concurrently within the Simscape plant model.

---------
<img width="1850" height="727" alt="image" src="https://github.com/user-attachments/assets/0d61711c-ff64-4f4d-9bb9-45d80195282a" />
Figure 2 The excitation experiment configuration .

-----------------

<img width="1716" height="707" alt="image" src="https://github.com/user-attachments/assets/100674f8-73a5-4767-921e-5aec86adb8b0" />

Figure 3 The modeled friction torque

-------------------

<img width="1280" height="704" alt="Step_3_ParameterExcitation-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/89f6b4d1-7e9a-47e0-a9f6-fafdf2d8354a" />

Figure 4  A visualization of the excitation step

-----------


### 4. Data Extraction, Decimation & Windowing (Step_4)
Raw sensor captures output enormous high-frequency time-series datasets that strain processor memory allocations. To manage this data pipeline efficiently, the framework processes the logged variables using a strict sequence: **Decimation → Filtering → Data Cropping → Observation Matrix Construction**.

```text
[Raw Simscape Capture] 
         |
         v
 1. DATA DECIMATION       <-- Downsamples data density to preserve processor memory
         |
         v
 2. DIGITAL FILTERING     <-- Executes zero-phase forward-backward filtfilt loops
         |
         v
 3. DATA CROPPING         <-- Throws away artificial startup/shutdown edge spikes
         |
         v
 4. REGRESSOR ASSEMBLY    <-- Constructs uncontaminated full-rank observation matrix (Yc)
```

#### The Operational Importance of Data Cropping Post-Filtering
A critical phase of data conditioning involves applying a localized boundary window (`crop_idx = 100`) to completely discard the initial and final chunks of the execution dataset. When running zero-phase forward-backward digital filtering architectures (`filtfilt`), the filter requires a startup period to settle. This mathematical initialization introduces massive, unstable transient edge spikes at the absolute start and end boundaries of the processed array.

```text
  [ Filter Startup Spike ]                                             [ Filter Shutdown Spike ]

         |                                                                        |
         v                                                                        v

   |--- TRUNCATED ---|================= ACTIVE OPTIMIZATION DATA ==================|--- TRUNCATED ---|
   0            crop_idx                                                       l-crop_idx            l
```

Because of this, data cropping is strictly executed **after the filtering stage** to throw away these spiked, artificial data points before they can corrupt the system. If these boundary artifacts are not thrown away, passing the filtered position states ($q$) straight into subsequent numerical differentiation loops causes them to blow up into extreme gradients that distort joint velocity ($\dot{q}$) and acceleration ($\ddot{q}$) profiles. 

Leaving these boundary segments un-cropped forces the optimization engine to fit unphysical outliers, breaking the convergence of the structural regressor matrix ($Y_c$). Truncating these segments after the filtering loop ensures that your subsequent parameter identification loops rely entirely on clean, settled steady-state state trajectories.


### 5. High-Fidelity Data Filtering & Conditioning (Step_5)
Unlike noise-free variants, parameter identification scripts fail when processing raw, noisy sensor data. Step_5 runs a filtered position and raw torque approach to condition the data matrix.

```text
[Raw Position q] ---> [Zero-Phase Butterworth Filter] ---> [Central Difference #1] ---> [Velocity q_dot]
                                                                                            |
                                                                                            v
[Filtered Acceleration q_ddot] <------------------------------------------ [Central Difference #2]
```

#### Why Raw Torque Data is Retained for Estimation
While the kinematic joint positions ($q$) require filtering to allow noise-free velocity ($\dot{q}$) and acceleration ($\ddot{q}$) differentiation, the identification target vector relies strictly on **raw, unconditioned torque data ($\tau_{raw}$)**. 

Filtering the measured torque channels introduces critical operational risks to the model-based engine:
* **Preserving True High-Frequency Dynamics**: Low-pass filtering the torque can accidentally attenuate high-frequency rigid-body torque details, unmodeled joint friction signatures, and physical unmodeled chatter. 
* **Preventing Parameter Phase Lag Distortion**: Even zero-phase filtering (`filtfilt`) alters the energy landscape of structural transient torques. Matching filtered kinematics against filtered torques can cause the optimization engine to output skewed physical parameter results.
* **Refining the Residual Noise Variance**: Retaining the unconditioned raw torque allows the convex optimization routine (`fmincon`) to process the authentic signal environment, mapping structural link attributes accurately while allowing the objective function to cleanly isolate and reject high-frequency zero-mean sensor noise.


### 6. Independent Trajectory Cross-Validation
To guarantee that the estimated parameter model does not overfit to the initial identification trajectory, the framework implements a strict multi-experiment cross-validation pipeline (`Step_6` through `Step_8`). 

The identified model parameters are tested against an entirely independent validation trajectory profile featuring distinct amplitude profiles, frequencies, and workspace pathways. By driving the model-based reconstruction equations using this unseen dataset and checking estimated torque tracking against independent experimental recordings, the framework confirms that the parameter vector maps true structural hardware realities rather than single-trajectory behavior.


---------------
<img width="1917" height="918" alt="image" src="https://github.com/user-attachments/assets/82a2b9f7-5c0b-411c-a237-619c27e82831" />
Figure 5 A slow trajectory validation.

-----------
| Experiment element | Metric | Value |
| :--- | :--- | :--- |
| **Joint 1 (Base Yaw)** | Fit Percentage | 99.12 % |
| **Joint 2 (Shoulder)** | Fit Percentage | 99.48 % |
| **Joint 3 (Elbow)** | Fit Percentage | 98.72 % |
| **OVERALL CROSS-VALIDATION ACCURACY** | | **99.54 %** |

------------
<img width="1917" height="923" alt="image" src="https://github.com/user-attachments/assets/c3d03bd5-51cd-40f5-8c4d-4eeb1b130cac" />
Figure 6 A fast and aggressive trajectory validation

----------



| Experiment element | Metric | Value |
| :--- | :--- | :--- |
| **Joint 1 (Base Yaw)** | Fit Percentage | 99.22 % |
| **Joint 2 (Shoulder)** | Fit Percentage | 99.41 % |
| **Joint 3 (Elbow)** | Fit Percentage | 98.58 % |
| **OVERALL CROSS-VALIDATION ACCURACY** | | **99.44 %** |










### 7. Physical Consistency & Epsilon ($\varepsilon$) Sensitivity Analysis
To keep parameters physically realistic, the `fmincon` objective function evaluates tracking error alongside strict structural constraints. It evaluates eigenvalues over a $5 \times 5 \times 5$ configuration grid (125 distinct workspace poses) to enforce a strictly positive-definite Mass/Inertia matrix ($M(q) > 0$).

The user-tunable boundary threshold value `epsilon_val` acts as a crucial sensitivity parameter:
* **High Epsilon Threshold ($\varepsilon \ge 0.05$)**: Strongly guarantees physical conservatism and numerical stability, preventing matrix division spikes during high-gain control. However, setting it too high can over-constrain the solver, causing poor data-fitting accuracy.
* **Low Epsilon Threshold ($\varepsilon \le 0.01$)**: Minimizes torque reconstruction errors by giving the solver more freedom, but risks approaching structural singularities where the mass matrix loses its positive-definiteness ($M(q) \to 0$).

This repository leverages an optimized balance of `epsilon_val = 0.037`, protecting model-based controllers from inverse-matrix spikes under feedback noise.

### 8. Robust Computed Torque Control Subsystem (Step_11)
The identified dynamic parameter matrix components directly feed the model-based controller. The system processes feedforward nonlinear rigid-body forces under high tracking feedback noise according to the following control law formulation:

$$ \tau = M(q)\left[\ddot{q}_d + K_p e + K_d \dot{e}\right] + C(q, \dot{q})\dot{q} + G(q) + F(\dot{q}) $$

Where $e = q_d - q$ represents the localized tracking joint error vector loop array driving system errors to zero.

Where \(e = q_d - q\) represents the localized tracking joint error vector loop array driving system errors to zero.



```

$$
\begin{array}{c}
q_d, \dot{q}_d, \ddot{q}_d \\
\downarrow \\
\mathbf{[Tracking~Error~Loop]} \xrightarrow{\quad u \quad} \mathbf{[Inverse~Dynamics~Model]} \xrightarrow{\text{Joint Torques}} \mathbf{[Simscape~Robot]} \\
(\theta_b) \\
\end{array}
$$

$$
\nwarrow \quad \text{Feedback } (q, \dot{q}) \quad \swarrow
$$

.....

<img width="1870" height="722" alt="image" src="https://github.com/user-attachments/assets/9b470e94-18e4-4dbe-bda7-7cc80cd3d1fd" />

Figure 7 : The computeed torque controller RRR robot arm configuration surface level

---------
<img width="1405" height="731" alt="image" src="https://github.com/user-attachments/assets/ea83d2de-6d63-4b2f-838a-9a8648f3f857" />

Figure 8 : The construction of the computed torque controller 



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
├── Step_8_Validation_of_Estimation.m       <-- Multitarget cross-validation verification tool
├── Step_9_ReformTheEstimatedMatrices.m     <-- Matrix decoupling reformer
├── Step_10_CheckPositive.m                 <-- 27,000-point physical QA guard
├── Step_11_Results.m                       <-- Metrics reporting and plotting utility
├── Step_11_TheEstimatedINVDynamicsMatricesCTC.slx <-- Closed-loop noisy CTC tracking model
├── get_RRR_M_sfun.m                        <-- S-function block for Inertia Matrix evaluation
├── get_RRR_C_sfun.m                        <-- S-function block for Coriolis Matrix evaluation
└── get_RRR_G_sfun.m                        <-- S-function block for Gravity Vector evaluation
```

---

## ⚠️ Hardware & Memory Constraints Note

Symbolically evaluating or validating a full joint-space mass matrix $M(q)$ across dynamic trajectories can introduce severe RAM allocations and processing delays. To prevent memory stack overflows, ensure your development environment has at least 16 GB of RAM when running the 27,000-point physical QA guard (`Step_10_CheckPositive.m`).

---

## 🚀 Getting Started & Execution Guide

To run the complete framework pipeline from scratch, follow these execution phases step-by-step:

### Phase 1: Structural Setup & Path Generation
1. **Run `Step_1_TheRegressorModel.m`**: Parses your proximal Denavit-Hartenberg structures to construct and write the symbolic regressor matrix files to disk.
2. **Run `Step_2_excitation_trajectory.m`**: Launches the bounded multi-harmonic optimization setup to output your targeted system trajectories.

### Phase 2: Plant Simulation & Data Cleaning
3. **Open & Run `Step_3_ParameterExcitation.slx`**: Activates the Simscape mechanics solver loop to log active torques against injected sensor tracking noise.
4. **Open & Run `Step_4_DataextractForParameterEstimation.slx`**: Truncates edge transients via a 100-sample window baseline and decimates high-frequency datasets.
5. **Run `Step_5_Filtered_Parameter_estimation.m`**: Drives the forward-backward Butterworth filtering engine and runs the constrained optimization solver with $\varepsilon = 0.037$.

### Phase 3: Validation, Verification & Control Execution
6. **Open & Run `Step_6_testingTheEstimatedParameters.slx`**: Feeds a completely separate, independent cross-validation trajectory profile into the Simscape plant model.
7. **Open & Run `Step_7_DataextractForSecondExpValidation.slx`**: Segregates, saves, and compiles the validation sensor tracking responses from this distinct layout.
8. **Run `Step_8_Validation_of_Estimation.m`**: Cross-validates mathematical fit quality by plotting estimated parameter torque reconstructions directly against raw data from the second experimental path profile.
9. **Run `Step_9_ReformTheEstimatedMatrices.m`**: Decouples the final optimized parameter arrays back into distinct $M, C, G,$ and $F$ matrices.
10. **Run `Step_10_CheckPositive.m`**: Loops across a dense 27,000-point workspace coordinate array to assert positive-definiteness ($M(q) > 0$).
11. **Run `Step_11_Results.m`**: Compiles evaluation metrics and builds performance comparison plots.
12. **Open & Run `Step_11_TheEstimatedINVDynamicsMatricesCTC.slx`**: Implements the identified system matrices using S-function interfaces (`get_RRR_M_sfun.m`, `get_RRR_C_sfun.m`, `get_RRR_G_sfun.m`) inside the Computed Torque Controller block loops to check trajectory tracking error decay under active feedback noise.
