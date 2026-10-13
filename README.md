# G1 Estimation-Aware Locomotion

<div align="center">

![G1 Humanoid](docs/images/g1_hero.png)

**Removing privileged base-velocity observations from bipedal locomotion reinforcement learning**

[![Isaac Lab](https://img.shields.io/badge/Built%20on-Isaac%20Lab-orange)](https://isaac-sim.github.io/IsaacLab/)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Status-Research%20in%20Progress-yellow)](#project-status)

</div>

---

## Abstract

This project investigates the effect of removing privileged base-velocity observations from bipedal locomotion reinforcement learning in NVIDIA Isaac Lab. Standard velocity-tracking locomotion pipelines often provide the policy with ground-truth base velocity computed by the physics engine. Because physical robots cannot directly measure this quantity, this project replaces the privileged observation with an estimated base-velocity signal derived from IMU measurements, leg kinematics, and foot-contact observations.

The policy receives IMU-derived angular velocity, projected gravity, biased linear acceleration, noisy binary foot-contact observations, joint positions, joint velocities, and a base-velocity estimate computed by a per-axis linear Kalman filter. The filter integrates gravity-compensated IMU acceleration and applies stance-leg odometry corrections using foot-contact constraints and leg Jacobians.

The current implementation removes direct ground-truth base velocity from the policy observation space. It does not yet remove all simulator-provided quantities from the estimation pipeline: the current leg-odometry calculation uses simulator-provided base position and IMU orientation. Removing these remaining privileged signals is a primary next step.

An earlier flat-terrain experiment using a linear Kalman-filtered velocity estimate produced a timeout-surviving gait without direct privileged base-velocity observations. The current noisy-sensor and rough-terrain configuration remains under development.

---

## Project Status

| Capability | Status | Evidence |
|---|---|---|
| Privileged base-velocity observation removal | ✅ Implemented | `g1_observations.py`, `G1RoughEnvCfg` |
| IMU-based angular-velocity observation | ✅ Implemented | `imu_ang_vel` |
| IMU-based projected-gravity observation | ✅ Implemented | `imu_projected_gravity` |
| Accelerometer-bias injection | ✅ Implemented | `imu_lin_acc_biased`, `reset_imu_bias` |
| Noisy foot-contact observations | ✅ Implemented | `foot_contact_booleans` |
| IMU-plus-leg-odometry velocity estimation | ✅ Implemented | `estimated_base_lin_vel` |
| Fully onboard-equivalent state estimation | ⏳ Not yet implemented | Simulator orientation and base position remain in estimator |
| Accelerometer-bias estimation | ⏳ Not yet implemented | Bias is injected but not estimated |
| Flat-terrain locomotion baseline | ✅ Demonstrated | Earlier Kalman-filter experiment |
| Rough-terrain locomotion | 🔄 Under development | Rough-terrain configuration exists; result not established |
| Cross-simulator validation | ⏳ Planned | Not implemented in current results |
| Physical G1 deployment | ⏳ Not currently available | No hardware deployment demonstrated |

---

## Demo

<div align="center">

![Flat-terrain gait](docs/videos/ImprovedGait.gif)

*Flat-terrain gait trained using Kalman-filtered velocity estimates rather than direct privileged base velocity.*

![Rough-terrain gait](docs/videos/RoughHop.gif)

*Rough-terrain training demonstration. The current policy exhibits hopping behavior and requires retraining.*

</div>

The demonstrations are qualitative. They should not be interpreted as evidence of completed rough-terrain generalization or physical deployment.

---

## Overview

This repository is a custom manager-based reinforcement-learning task extension for [Isaac Lab](https://isaac-sim.github.io/IsaacLab/). It adapts Isaac Lab’s velocity-tracking locomotion pipeline for the Unitree G1 humanoid.

The central modification is the replacement of the simulator’s ground-truth base-linear-velocity observation with an estimated velocity signal. The estimator fuses:

- Gravity-compensated IMU linear acceleration.
- Stance-foot velocity constraints derived from leg Jacobians and joint velocities.
- Binary foot-contact observations derived from contact-force thresholds.

The policy observes the estimated base velocity rather than the simulator’s privileged base velocity.

---

## Contributions

| Component | Origin | G1 modification |
|---|---|---|
| Velocity-tracking locomotion environment | Isaac Lab | Estimation-aware observation replacement |
| IMU sensor | Isaac Lab | Added to `torso_link`; used for proprioceptive observations |
| Base-velocity estimation | Original implementation | Per-axis Kalman filter with IMU integration and stance-leg odometry |
| Contact observation | Isaac Lab contact sensor | Thresholded binary contacts with label noise |
| Accelerometer bias | Original implementation | Persistent per-episode bias injection |
| Gait shaping | Original implementation | Foot-phase and foot-height reward terms |
| Rough-terrain configuration | Isaac Lab base environment | Adapted reward, observation, and reset configuration |

---

## State-Estimation Pipeline

The standard Isaac Lab velocity-tracking pipeline provides the policy with ground-truth base linear velocity computed by the physics engine. This project removes that privileged observation and replaces it with an estimated base-velocity signal derived from IMU measurements, joint states, and foot-contact observations.

The policy does not directly observe the simulator’s ground-truth base linear velocity. However, the current estimator still uses simulator-provided IMU orientation and base position. Removing these remaining privileged signals is a primary next step toward a fully onboard-equivalent observation pipeline.

### Observation Replacement

The privileged base-linear-velocity observation is removed:

```python
self.observations.policy.base_lin_vel = None
```

It is replaced by an estimated base-linear-velocity observation:

```python
self.observations.policy.estimated_base_lin_vel
```

The policy observation therefore contains an estimate of base velocity rather than the simulator’s ground-truth value.

### Sensor Configuration

An IMU sensor is attached to the robot’s torso link:

```python
self.scene.imu = ImuCfg(
    prim_path="{ENV_REGEX_NS}/Robot/torso_link",
)
```

The estimator and policy use the following observations:

| Observation | Source | Noise model |
|---|---|---|
| Angular velocity | IMU body-frame angular velocity | Uniform, ±0.05 rad/s |
| Projected gravity | IMU-derived gravity projection | Uniform, ±0.02 |
| Linear acceleration | IMU body-frame acceleration plus persistent bias | Uniform, ±0.2 m/s² |
| Foot contacts | Contact-force thresholding | Independent 5% label flipping |
| Joint positions | Robot joint state | Inherited environment configuration |
| Joint velocities | Robot joint state | Inherited environment configuration |
| Estimated base velocity | Kalman-filtered IMU and leg-odometry fusion | Derived estimate |

### Accelerometer Bias

A persistent accelerometer bias is injected into the IMU linear-acceleration observation. The bias is sampled once per environment reset from a zero-mean normal distribution with standard deviation 0.15:

$$
b_a \sim \mathcal{N}(0, 0.15^2 I_3)
$$

The biased acceleration observation is:

$$
\tilde{a}^b_k = a^b_k + b_a
$$

where:

- $`\tilde{a}^b_k`$ is the biased body-frame linear acceleration.
- $`a^b_k`$ is the simulator IMU body-frame linear acceleration.
- $`b_a`$ is the persistent accelerometer bias.

The current filter does **not** estimate or compensate for this bias. Bias estimation is a planned extension.

### Foot-Contact Observations

Foot-contact flags are computed by thresholding the norm of the net contact force acting on each ankle-roll link:

$$
c_i =
\begin{cases}
1, & \lVert f_i \rVert > 5.0\ \text{N} \\
0, & \text{otherwise}
\end{cases}
$$

where:

- $`c_i`$ is the binary contact flag for foot $`i`$.
- $`f_i`$ is the net contact force on foot $`i`$.
- The threshold is 5.0 N.

Each binary contact label is independently flipped with probability 0.05 at each observation step:

$$
\tilde{c}_i =
\begin{cases}
1 - c_i, & \text{with probability } 0.05 \\
c_i, & \text{otherwise}
\end{cases}
$$

This models observation noise rather than persistent contact-state misclassification. The contact sensor reports the net contact force acting on each monitored rigid body.

---

## Velocity Estimation

The velocity estimator is a **per-axis linear Kalman filter**. It maintains a three-dimensional velocity estimate and a three-dimensional diagonal covariance estimate.

It is not an extended Kalman filter: the current implementation does not estimate attitude, accelerometer bias, or gyroscope bias, and it does not linearize a nonlinear state-transition model.

### Prediction Step

At each environment step, the filter predicts velocity by integrating gravity-compensated IMU acceleration:

$$
\hat{v}^{-}_{k+1} = \hat{v}_{k} + \left(R(q_k)\tilde{a}^{b}_{k} + g\right)\Delta t
$$

The covariance prediction is:

$$
P^{-}_{k+1} = P_{k} + Q\Delta t
$$

where:

| Symbol | Meaning |
|---|---|
| $`\hat{v}_{k}`$ | Previous velocity estimate |
| $`\hat{v}^{-}_{k+1}`$ | Predicted velocity before measurement update |
| $`R(q_k)`$ | Rotation matrix from simulator-provided IMU orientation |
| $`\tilde{a}^{b}_{k}`$ | Biased body-frame linear acceleration |
| $`g`$ | World-frame gravity vector, $`[0,0,-9.81]`$ m/s² |
| $`\Delta t`$ | Environment time step |
| $`P_{k}`$ | Previous velocity covariance |
| $`P^{-}_{k+1}`$ | Predicted velocity covariance |
| $`Q`$ | Process-noise covariance |

The gravity-compensation term assumes that the IMU reports proper acceleration, including the reaction to gravity. Rotating the body-frame acceleration into the world frame and subtracting gravitational acceleration yields the linear-acceleration component used for velocity integration.

### Stance-Leg Odometry Measurement

For each foot detected in stance, the estimator computes the base velocity implied by the zero-velocity foot constraint:

$$
v_{\text{base},i} = -\omega_k \times r_{\text{foot},i} - J_{\text{foot},i}\dot{q}_k
$$

where:

| Symbol | Meaning |
|---|---|
| $`v_{\text{base},i}`$ | Base-velocity estimate implied by stance foot $`i`$ |
| $`\omega_k`$ | Base angular velocity |
| $`r_{\text{foot},i}`$ | Position of foot $`i`$ relative to the base |
| $`J_{\text{foot},i}`$ | Linear Jacobian of foot $`i`$ |
| $`\dot{q}_k`$ | Joint-velocity vector |

The base-velocity estimates from all active stance feet are averaged:

$$
v_{\text{leg}} = \frac{\sum_{i \in \mathcal{S}} v_{\text{base},i}}{|\mathcal{S}|}
$$

where $`\mathcal{S}`$ is the set of feet detected in stance.

If no foot is in stance, the leg-odometry correction is skipped and the filter propagates the prediction without a measurement update.

### Kalman Update

When at least one foot is detected in stance, the leg-odometry velocity is used as a direct measurement of base velocity. The measurement model is therefore $`H=I`$.

The measurement innovation is:

$$
r_k = v_{\text{leg}} - \hat{v}^{-}_{k+1}
$$

The Kalman gain is:

$$
K_k = \frac{P^{-}_{k+1}}{P^{-}_{k+1} + R}
$$

The velocity estimate is updated as:

$$
\hat{v}_{k+1} = \hat{v}^{-}_{k+1} + K_k r_k
$$

Equivalently, substituting the innovation:

$$
\hat{v}_{k+1} = \hat{v}^{-}_{k+1} + K_k\left(v_{\text{leg}} - \hat{v}^{-}_{k+1}\right)
$$

The covariance update is:

$$
P_{k+1} = \left(I - K_kH\right)P^{-}_{k+1}
$$

Because $`H=I`$, this reduces to:

$$
P_{k+1} = \left(1 - K_k\right)P^{-}_{k+1}
$$

where:

| Symbol | Meaning |
|---|---|
| $`r_k`$ | Measurement innovation |
| $`v_{\text{leg}}`$ | Leg-odometry velocity measurement |
| $`H`$ | Measurement model, equal to the identity matrix |
| $`K_k`$ | Kalman gain |
| $`\hat{v}^{-}_{k+1}`$ | Predicted velocity before measurement update |
| $`\hat{v}_{k+1}`$ | Post-update velocity estimate |
| $`P^{-}_{k+1}`$ | Predicted velocity covariance |
| $`P_{k+1}`$ | Post-update velocity covariance |
| $`R`$ | Measurement-noise covariance |

The implementation applies the update independently to each velocity axis. Equivalently, it uses a diagonal covariance matrix and assumes no cross-axis covariance between velocity components.

### Estimator Scope and Limitations

The current implementation has the following limitations:

- It is a per-axis linear Kalman filter, not an extended Kalman filter.
- It estimates velocity only.
- It does not estimate orientation, accelerometer bias, or gyroscope bias.
- It uses simulator-provided IMU orientation.
- It uses simulator-provided base position when computing the foot position relative to the base.
- It assumes fixed process and measurement noise.
- It uses independently flipped contact labels rather than a persistent contact-classification noise model.

The next development milestone is to remove simulator-provided orientation and base position from the estimator. A subsequent milestone is to extend the state to include attitude and accelerometer-bias states.

---

## Rewards

The reward function combines Isaac Lab’s velocity-tracking terms with custom bipedal gait-shaping terms.

| Reward term | Weight | Purpose |
|---|---:|---|
| Termination penalty | -200.0 | Discourage early termination |
| Linear-velocity tracking | 1.0 | Track commanded XY velocity |
| Yaw-rate tracking | 2.0 | Track commanded yaw rate |
| Foot air time | 0.25–1.0 | Encourage swing-phase timing |
| Foot sliding | -0.1 | Discourage stance-foot sliding |
| Ankle joint-limit penalty | -1.0 | Discourage ankle-limit violation |
| Hip-yaw/roll deviation | -0.1 | Regularize nonessential hip motion |
| Arm deviation | -0.1 | Keep arms near default configuration |
| Finger deviation | -0.05 | Keep fingers near default configuration |
| Torso deviation | -0.1 | Regularize torso motion |
| Foot-phase reward | 0.5 | Encourage alternating single-support contact |
| Foot-height reward | 0.3 | Encourage foot clearance during swing |
| Knee/hip-pitch deviation | -0.08 | Regularize leg configuration |

The foot-phase reward is maximized when exactly one foot is in contact. It decreases linearly for both double support and flight. The foot-height reward encourages swing-foot clearance up to a 5 cm target height.

---

## Environments

| Task | Terrain | Purpose | Status |
|---|---|---|---|
| `Isaac-Velocity-Flat-G1-v0` | Plane | Flat-terrain estimation-aware training | ✅ Implemented |
| `Isaac-Velocity-Flat-G1-Play-v0` | Plane | Deterministic flat-terrain evaluation | ✅ Implemented |
| `Isaac-Velocity-Rough-G1-v0` | Rough terrain | Rough-terrain estimation-aware training | 🔄 Under development |
| `Isaac-Velocity-Rough-G1-Play-v0` | Rough terrain | Deterministic rough-terrain evaluation | 🔄 Under development |

The rough-terrain configuration inherits Isaac Lab’s rough-terrain environment but reduces several forms of domain randomization. Push randomization, base-mass randomization, and base-center-of-mass randomization are disabled. Joint positions are reset to their default values, and initial joint velocities are set to zero.

The rough-terrain play configuration commands a fixed forward velocity of 1.0 m/s, disables observation corruption, and removes external force and push events.

---

## Results

### Flat-Terrain Baseline

An earlier flat-terrain experiment using the linear Kalman-filtered velocity estimate produced a timeout-surviving gait without direct privileged base-velocity observations.

The earlier reported 0.26 m/s velocity error should not be compared directly with current rough-terrain results unless the metric, terrain, sensor assumptions, and evaluation protocol are identical.

### Rough-Terrain Status

The current rough-terrain configuration exhibits unstable behavior, including hopping and early termination. This result is under investigation.

The current rough-terrain experiment should not be interpreted as a successful rough-terrain generalization result.

---

## Reproducibility

### Installation

```bash
pip install -e source/g1_locomotion
```

### Training

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Flat-G1-v0 \
  --headless
```

### Evaluation

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-G1-Play-v0 \
  --num_envs 50 \
  --checkpoint [FILL: checkpoint path]
```

---

## Disclaimer

This repository is intended for simulation-based robotics research. Trained policies and control code may produce unexpected or dynamically unstable behavior if transferred to physical hardware.

Use at your own risk. Validate thoroughly in simulation first, and use appropriate safety measures—including an accessible emergency stop, physical clearance, supervised operation, and protective equipment—before any physical deployment.

---

## References

- NVIDIA, “Isaac Lab,” NVIDIA Developer, 2024. [Online]. Available: [https://developer.nvidia.com/isaac/lab](https://developer.nvidia.com/isaac/lab)
- Isaac Lab Documentation, “Inertial Measurement Unit (IMU).” [Online]. Available: [https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/sensors/imu.html](https://isaac-sim.github.io/IsaacLab/main/source/overview/core-concepts/sensors/imu.html)
- Isaac Lab Documentation, “Contact Sensor.” [Online]. Available: [https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html)
- Isaac Lab Documentation, “Reinforcement Learning Scripts.” [Online]. Available: [https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html)
- Unitree, “G1 Humanoid Robot.” [Online]. Available: [https://www.unitree.com/g1](https://www.unitree.com/g1)
