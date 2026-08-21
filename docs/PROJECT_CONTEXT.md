# Project context

## Objective

Collect synchronized UR5 manipulation demonstrations for conversion to
LeRobot datasets and later training with SmolVLA, pi0, and compatible policies.

The recorder is deliberately separated from robot motion. It observes the
working joystick teleoperation stack and writes a stable staging format. It
never switches controllers, starts MoveIt Servo, or commands the UR5.

## Confirmed hardware and software

- Universal Robots UR5 at `10.0.1.38`
- ROS 2 Humble
- MoveIt 2 and MoveIt Servo
- Logitech Extreme 3D Pro joystick
- Base camera: Intel RealSense D435i, serial `044322072365`
- Wrist camera: Intel RealSense D435, serial `827312073590`
- Both cameras configured for USB 3, RGB, depth, and aligned depth-to-color

## Working control architecture

```text
Logitech joystick
  -> /joy
  -> logitech_servo.py
  -> /servo_node/delta_twist_cmds
  -> MoveIt Servo (differential inverse kinematics and safety)
  -> /forward_position_controller/commands
  -> forward_position_controller
  -> ur_robot_driver
  -> UR5
```

MoveIt Plan and Execute uses `scaled_joint_trajectory_controller`. Continuous
teleoperation uses `forward_position_controller`. Only the appropriate motion
controller should be active at a time.

## Recorded observations

- Base RGB image
- Wrist RGB image
- Base aligned depth image
- Wrist aligned depth image
- Six measured UR5 joint positions, reordered by joint name
- Estimated pneumatic-gripper state from the last successful SetIO command
- Natural-language task instruction
- Source timestamps and receive-age diagnostics

## Recorded action

The primary action is the six-element absolute joint-position target from
`/forward_position_controller/commands`, followed by a normalized binary
gripper command.

Raw joystick axes are not the learning action. MoveIt Servo changes the human
request through coordinate mapping, differential inverse kinematics, limits,
singularity handling, collision scaling, and smoothing. Recording the final
controller target preserves what the robot was actually asked to execute.

For model-specific preprocessing, arm deltas can later be derived as:

```text
delta_arm = q_command - q_actual
```

The raw staging files retain the absolute command as the source of truth.

## Canonical joint order

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

Never trust the incoming `/joint_states` array order. Reorder using the message
name field.

## Episode controls

The pneumatic gripper uses `0.0 = closed` and `1.0 = open`. Because it has no
position sensor, `/gripper/state` is explicitly an estimate of the last command
confirmed successful by the UR SetIO service. Both state and command are
republished at 20 Hz after initialization.

The episode joystick controls are rising-edge triggered:

- Physical button 9 / `buttons[8]`: start a new episode
- Physical button 10 / `buttons[9]`: abort and permanently discard the episode
- Physical button 11 / `buttons[10]`: stop and save the episode as successful

Equivalent ROS services remain available for start, stop, success, failure,
discard, and status operations.

## Data lifecycle

Episodes are written below `~/ur5_vla_dataset/staging/episodes/`. An active or
unlabelled episode uses a hidden `.incomplete` directory. Finalization validates
metadata and atomically renames the directory. Aborting removes it.

The staging format is intentionally independent of a particular LeRobot
release. Conversion to LeRobot should occur only after one episode passes
schema checks, timestamp checks, image/depth decoding, and a safety-limited
replay test from the same start pose.

## Deployment relationship

During policy deployment, the joystick mapper is replaced by a policy executor:

```text
camera observations + current robot state + task
  -> SmolVLA / pi0 / other policy
  -> policy executor and safety filter
  -> MoveIt Servo
  -> forward_position_controller
  -> UR5
```

Training and deployment must use the same joint order, units, camera keys,
gripper convention, normalization, and action meaning.
