<p align="center">
  <a >
    <img src="ext/image/logo.png" alt="logo" width="300">
  </a>
</p>

---

# mc_rtc RL-QP controller template

This project is a template for a new RL-QP controller project wihtin [mc_rtc].

The goal of this template is to help users deploy reinforcement learning (RL) policies within a Quadratic Programming (QP) framework augmented with Control Barrier Functions (CBFs). This combination is enforcing physical and safety constraints, including:

* Joint position limits
* Joint velocity limits
* Torque limits
* Self-collision avoidance

Further details are available in:

[*Safe Execution of RL Policies via Acceleration-based CBF-QP Constraint Enforcement for Real-World Robotic Deployments*](https://hal.science/hal-05362571)

It comes with:
- a CMake project that can build a controller in [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

Currently only ONNX format policies are supported.

This template has already been adapted to :
- [H1](https://github.com/Alhuuin/h1_rl_qp_controller)
- [HRP5P](https://github.com/bastien-muraccioli/hrp5p_rl_qp_controller)

If the target robot is already supported, please refer to the corresponding repository above. Otherwise, this template is intended as a starting point to integrate additional robots.

## Documentation

The complete documentation, including the controller architecture, observation system, policy configuration, adaptation guide and API reference, is available at:

**https://alhuuin.github.io/rl-qp-controller.github.io/**

The documentation also includes practical guides for adapting the controller to new robots and examples.

## Template layout

- `policies/minimalExample/`: smallest explicitly configured policy accepted by the current parser.
- `policies/fullExample/`: exhaustive reference for every currently accepted policy and observation fields.
- `policies/conventions.yaml`: placeholder joint groups, training orders, aliases and observation defaults.
- `etc/G1RLQPController.in.yaml`: controller, observer and constraint examples.
- `src/states/G1RLQPController_Initial.cpp`: policy-execution state.
- `src/observation/`: reusable observation implementations.
- `src/policy/`: policy loading, validation, ONNX inference and runtime state.

## Required robot adaptation

The source marks adaptation points with `TODO(robot)`. Search them before attempting to run the controller:

```bash
grep -rn "TODO(robot)"
```

## Deploy your own policy

Policies are stored in `policies`. Each subdirectory describes everything the controller needs to know to load. run and use the policy.
You here have access to 2 example directories. `minimalExample` with a basic, minimal configuration, and `fullExample` to have a better idea of the full potential of the default version of this controller.

1. Create a new subdirectory in `policies`
1. Add the exported model in ONNX format inside
2. set the action and controlled-joint groups, provide training-consistent `kp` and `kd`, and add any required action scaling, default pose, phase period or command parameters in `policy.yaml`
5. reproduce the exact observation order and history in `observations.yaml`.

Set `default_policy` in `etc/G1RLQPController.in.yaml`.

## Acceleration policies

Besides position-PD policies, the controller runs policies whose action is a joint acceleration (mjlab
`JointAccelerationActionCfg`), as trained in `mjlab_acceleration_policies`. In `policy.yaml`:

```yaml
action:
  type: acceleration        # default: position
  joints: all
  acceleration_limit: 900.0 # |qdd*| clamp, rad/s^2
  scale: {...}              # qdd* = scale * action
  q0: {...}                 # training default pose (joint_pos observation)
control:
  use_QP: true              # required: qdd* is only tracked inside the QP
  posture_weight: 10.0
```

`qdd*` is sent to the FSM `PostureTask` (zero gains, `refAccel`), which replaces the `TorqueJointTask` in the
CBF-QP while such a policy is active; `kp`/`kd` are not used. The QP output torques are sent to the robot as for
position policies (`ControlMode: Torque`).

These policies also observe `qacc_ext = M^-1 tau_ext` (observation type `qacc_ext`), computed from the
`ExternalForcesObserver` estimate in MuJoCo's free-joint layout `[base linear (world), base angular (base),
joints]`, and the estimated base linear velocity (`base_lin_vel` with `body: base`). On hardware the base state
must come from the Tilt observer: `etc/G1RLQPController.in.yaml` is set up for that.

| policy folder | training | observes `qacc_ext` |
|---|---|---|
| `arm_wrench_joint_dr_17998` | ArmWrenchRamp + joint-friction randomization (default, deployment candidate) | yes |
| `arm_wrench_ramp_14999` | ArmWrenchRamp, seed 42 | yes |
| `arm_wrench_ramp_tilt_ft_17998` | ArmWrenchRamp fine-tuned with the Tilt observer in the loop | yes |
| `arm_wrench_ramp_s43_14999` | ArmWrenchRamp, seed 43 | yes |
| `arm_wrench_ramp_tilt_ft_s43_17998` | Tilt fine-tune of seed 43 | yes |
| `arm_wrench_ramp_no_qacc_14999` | ablation without `qacc_ext` | no |
| `raphael_08_13_21550` | Raphaël Barrabes's 2026-08-13 policy | no |

The joint-limit CBF gains used to validate them are in the `joint_limit_cbf` entry of the configuration.

Checked against the previous G1RLQPController in mc_mujoco (2026-11-07, `arm_wrench_joint_dr_17998`, mjlab-matched
G1 model): straight walking 0.46-0.47 m/s at a 0.5 m/s command; no fall on turns, command schedule or 150 N pushes,
with ground truth and with the Tilt observer on a noisy IMU; 300 N pushes at the CoM 0.30 falls/min with ground
truth and 0.83 with Tilt + noise (3 falls each; previous controller 0.31 and 0.65 with 6 falls).

For simulation tests, an optional `sim_test` entry gives a velocity command schedule and random pushes through
mc_mujoco (see `loadSimTest()` in `G1RLQPController.h`).

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j
sudo cmake --install .
```

Enable your controller in mc_rtc after completing all required adaptation points.

## Control flow summary

```
Every controller timestep (physics_step_size):
├── If syncTime >= policyStepSize:
│   ├── observation = getCurrentObservation()
│   ├── action      = rlPolicy->predict(observation)
│   ├── q_rl        = action * actionScale + q_zero
│   └── syncTime    = 0
│
├── τ = Kp*(q_rl - q) - Kd*q̇
│
├── useQP=true:  τ → TorqueJointTask → CBF-QP → robot
└── useQP=false: τ → robot (direct)

Acceleration policies (action.type: acceleration):
├── qdd* = clamp(action * actionScale, ±acceleration_limit), held between policy steps
└── qdd* → PostureTask refAccel → CBF-QP → τ → robot
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
