# UR5 VLA Dataset Recorder

This ROS 2 Humble package records synchronized UR5 demonstrations into a stable staging format. It does not command the robot. It samples:

- `/base/camera/color/image_raw`
- `/wrist/camera/color/image_raw`
- `/base/camera/aligned_depth_to_color/image_raw`
- `/wrist/camera/aligned_depth_to_color/image_raw`
- `/joint_states`, reordered by joint name
- `/forward_position_controller/commands`, the final MoveIt Servo arm action
- `/servo_node/status`
- normalized pneumatic-gripper state and command

The current default is 20 Hz with both RealSense cameras connected over USB 3. RGB is stored as JPEG; aligned metric depth is stored losslessly as 16-bit PNG. The pneumatic-gripper convention is `0.0 = closed` and `1.0 = open`.

## Install dependencies

```bash
conda deactivate
source /opt/ros/humble/setup.bash
sudo apt install python3-opencv python3-yaml ros-humble-cv-bridge
```

## Build

Copy this package into the `src` directory of a ROS 2 workspace, then:

```bash
cd ~/ur5_vla_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ur5_vla_dataset_recorder
source install/setup.bash
```

## Verify the live streams

Launch each USB 3 camera with aligned depth enabled. Base D435i:

```bash
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_044322072365 \
  camera_namespace:=base \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

Wrist D435:

```bash
ros2 launch realsense2_camera rs_launch.py \
  serial_no:=_827312073590 \
  camera_namespace:=wrist \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
```

Verify that the installed wrapper created the exact configured topics:

```bash
ros2 topic hz /base/camera/color/image_raw
ros2 topic hz /wrist/camera/color/image_raw
ros2 topic hz /base/camera/aligned_depth_to_color/image_raw
ros2 topic hz /wrist/camera/aligned_depth_to_color/image_raw
ros2 topic type /base/camera/aligned_depth_to_color/image_raw
ros2 topic type /wrist/camera/aligned_depth_to_color/image_raw
ros2 topic echo /joint_states --once
ros2 topic echo /forward_position_controller/commands --once
ros2 topic echo /servo_node/status --once
```

If the Servo status topic is not `std_msgs/msg/Int8`, check it with:

```bash
ros2 topic type /servo_node/status
```

The node currently expects `std_msgs/msg/Int8` on that topic.

## Run

Start the repository-maintained joystick mapper instead of the older standalone
script:

```bash
ros2 run ur5_vla_dataset_recorder logitech_servo
```

The mapper preserves the original joystick motion and UR digital-output
behavior. After an OPEN or CLOSE `SetIO` request succeeds, it publishes both:

```text
/gripper/command  std_msgs/msg/Float64
/gripper/state    std_msgs/msg/Float64
```

`/gripper/state` is the last successfully confirmed command, not a measured
finger position: the current pneumatic gripper has no position sensor. Operate
the gripper once before starting the recorder so its initial state is known.

Verify the values:

```bash
ros2 topic echo /gripper/command
ros2 topic echo /gripper/state
```

Then start the recorder:

```bash
ros2 launch ur5_vla_dataset_recorder recorder.launch.py
```

In another terminal, set the episode instruction:

```bash
ros2 param set /dataset_recorder task \
  "Pick up the red cube and place it in the tray"
```

Check readiness:

```bash
ros2 service call /dataset_recorder/status std_srvs/srv/Trigger "{}"
```

Start and stop one episode:

```bash
ros2 service call /dataset_recorder/start_episode std_srvs/srv/Trigger "{}"

# Teleoperate the complete attempt.

ros2 service call /dataset_recorder/stop_episode std_srvs/srv/Trigger "{}"
```

## Joystick episode buttons

The default Logitech mapping is edge-triggered, so holding a button does not
repeat the operation:

| Physical label | `/joy` index | Recorder action |
|---|---:|---|
| Button 9 | `buttons[8]` | Start a new episode |
| Button 10 | `buttons[9]` | Stop and permanently discard/abort the current episode |
| Button 11 | `buttons[10]` | Stop and save the current episode as successful |

Set the `task` parameter before pressing button 9. Button 9 refuses to replace
an active or unlabeled episode. Button 10 also discards an episode that was
already stopped but not labeled. Button 11 can save an active episode or one
that was already stopped.

Confirm the indexes on your joystick before collecting real data:

```bash
ros2 topic echo /joy
```

If the physical labels use different indexes, edit `config/recorder.yaml`.
The service commands remain available as a backup.

Label it using exactly one command:

```bash
ros2 service call /dataset_recorder/save_success std_srvs/srv/Trigger "{}"
ros2 service call /dataset_recorder/save_failure std_srvs/srv/Trigger "{}"
ros2 service call /dataset_recorder/discard_episode std_srvs/srv/Trigger "{}"
```

Data is saved under `~/ur5_vla_dataset/staging/episodes/`. Until an episode is labeled, its directory starts with a dot and ends in `.incomplete`. Finalization is an atomic rename.

## Frame schema

Each `frames.jsonl` row contains:

```text
observation.images.base      relative JPEG path
observation.images.wrist     relative JPEG path
observation.depth.base       relative lossless 16-bit PNG path
observation.depth.wrist      relative lossless 16-bit PNG path
observation.state            six actual UR5 joints + estimated binary gripper state
action                       six absolute joint targets + binary gripper command
task                         episode instruction
episode_time_s               monotonic time from episode start
source timestamps            camera and joint message timestamps
receive_age_s                freshness diagnostic for every stream
servo_status                 raw status code
valid                        true for accepted frames
```

Raw `/joy` values are intentionally not used as the learning action. The saved arm action is the final joint-position target produced by MoveIt Servo.

The gripper values make each state and action seven-dimensional. The mapper
publishes them at 20 Hz after the first successfully confirmed open or close
operation so the recorder freshness gate remains meaningful.

Depth PNG values are raw `uint16`. With the default D435/D435i configuration,
convert a valid pixel to metres using:

```text
depth_metres = depth_png_value * 0.001
```

Zero normally represents invalid/missing depth. The exact scale is saved as
`depth_unit_m` in `metadata.yaml` and should be verified against the live camera
configuration before a final dataset collection.

Depth consumes substantially more disk than RGB-only recording. The default
minimum-free-space gate is therefore 20 GB. Monitor the dataset size during the
first test episode with `du -sh ~/ur5_vla_dataset`.

## Safety behavior

- Recording cannot start without all mandatory fresh streams and a non-empty task.
- With the default configuration, Servo status must be `0`.
- Missing joints, duplicate joints, wrong command length, NaN, infinity, stale messages, writer errors, and low disk space prevent or drop frames.
- Recording cannot start until one gripper operation has succeeded and fresh normalized gripper state/command values are available.
- Disk writing occurs on a bounded background queue.
- The recorder never switches controllers, starts Servo, or commands the robot.
- Always stop an episode before saving or discarding it.

Keep a parallel rosbag2 backup during early development. Never replay a bag while a physical command controller is active.
