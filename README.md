# IL_Recorder

A general purpose data recorder for imitation learning with ROS2. 

Designed to allow use across multiple robots, with the only requirement needed being to specify a robot config file and adapter to handle the data types used on that robot.

# Install

Simply clone this package into your ros2 workspace, and build with the standard `colcon build --symlink-install --packages-select il_recorder`.

## Dependencies
Requires the following python packages:
- `numpy`
- `scipy`
- `fpsample`
- `h5py`

# Usage Guide

Start the recorder by running the ROS 2 node and passing the paths to your configuration files.

## Running the Recorder

```bash
ros2 run <package_name> il_recorder \
    --ros-args \
    -p robot_config:=fr3.yaml \
    -p obs_config:=all.yaml \
    -p base_path:=data/my_experiment
```

The recorder will save hdf5 files with your data. To check the contents of them, you can use: `ros2 run il_recorder inspect_hdf5 <path>` 

## Control

The recorder listens to the /joy topic (standard Gamepad messages).

- Start Recording: Button 7 (Start/Options). Creates a new HDF5 file and begins buffering frames.

- Stop & Save: Button 7 (Start/Options) (while recording). Stops recording, drains the processing queue, and saves to disk.

- Cancel/Discard: Button 8 (Select/Share). Stops recording immediately and discards all buffered frames.

## Debugging Pointclouds

Since it can be hard to verify your pointcloud parameters, the package also contains a script to debug and visualize your processed pointclouds.

Run:

```bash
ros2 run il_recorder pc_check \
    --ros-args \
    -p robot_config:=wam7.yaml \
    -p obs_config:=all.yaml
```

The cropped and downsampled pointcloud is then published so that it can be seen in RViz, letting you confirm if it is capturing the correct parts of the scene.

# Customizing the Recorder

The recorder is designed such that you can add functionality for a new robot, or change what you save for an existing robot by doing 3 steps:
1. Write an adapter that defines what you want to save
2. Specify a robot config file that tells the recorder what to subscribe to
3. (Optionally) define an observation config file to modify what point cloud and image modalities we save

## 1. Adapters (`adapters.py`)

If you need to add a new robot, you'll want to write an adapter. These functions how we convert low dimensional (position, joints, gripper...) data to a dictionary to save. The recorder node itself will handle passing the data to your adapter, based on the config files (described below); so you only need to modify this if you need to add a new robot. 

Your adapter should look like the following:

```python
def adapter(msg_dict, params: Dict[str, Any]):
```

Once finished, add the adapter to the ADAPTERS dictionary at the bottom of the file.

See below (Robot Configuration) to see how you can specify what will be passed to the adapter's `msg_dict` and `params`.

Your adapter will be selected based on the `adapter_type` argument in the robot config (discussed below). If not given, it defaults to `robot_name`.

You can use the `adapter_type` argument to allow for different adapters for the same robot if you just need a different data format. 

## 2. Robot Configuration (`configs/robots`)

This file maps ROS topics to internal keys and provides hardware parameters to the Python adapter. It defines *where* the data comes from. 

## Structure

```yaml
# Unique identifier for the robot (stored in HDF5 metadata)
robot_name: "fr3" 

# (Optional) Specific adapter function to use. 
# If omitted, defaults to looking up 'robot_name' in ADAPTERS dictionary.
adapter_type: "fr3"

# List of ROS topics to subscribe to
active_subscribers:
  # 'key':  Internal identifier used by the adapter and recorder.
  # 'topic': The ROS topic name.
  # 'type':  The full ROS message type string (e.g., package/msg/Type).
  
  # --- Standard Low-Dim Keys (Passed to Adapter) ---
  # These keys are aggregated and passed to your custom adapter function.
  - {key: "joints",    topic: "/fr3/joint_states",       type: "sensor_msgs/msg/JointState"}
  - {key: "cartesian", topic: "/fr3/end_effector_pose",  type: "geometry_msgs/msg/PoseStamped"}
  - {key: "gripper",   topic: "/fr3/gripper_state",      type: "franky_msgs/msg/GripperState"}
  
  # --- Reserved Modality Keys (Controlled by obs_config) ---
  # The recorder looks specifically for keys containing "image" and "pointcloud".
  # Multiple cameras are supported as long as you ensure all keys contain the words image or pointcloud.
  # These are ONLY subscribed to if enabled in the observation config.
  - {key: "image",      topic: "/camera/color/image_raw",       type: "sensor_msgs/msg/Image"}
  - {key: "pointcloud", topic: "/camera/depth/color/points",    type: "sensor_msgs/msg/PointCloud2"}

# Arbitrary parameters passed to the Python Adapter function
params:
  dof: 7
  has_gripper: true
```

**Anything in params** will be passed into the adapter function, so you can use this to specify arbitrary robot specific config.

---

### 3. Observation config (`configs/observations`)

This file controls the "high-dimensional" data pipeline. It determines *what* is recorded and applies pre-processing (cropping, resizing) before saving to disk.

Currently, only DP3 (crop + farthest point sample) and iDP3 (voxel downsample + uniform sample) are available for downsampling.

Similarly, for images, only resizing is currently available. 

For other processing, either modify the code, or save without processing and then run your postprocessing later.

## Point Clouds

```yaml
pointcloud:
  enabled: true  # If false, the 'pointcloud' subscriber is ignored
  
  # Processing Mode
  # Options: 
  #   - "raw":  Save flattened arrays without modification.
  #   - "dp3":  Fixed workspace crop + farthest point sampling (Diffusion Policy 3D).
  #   - "idp3": Depth-based crop + voxel sampling (Improved DP3).
  processing_type: "dp3" 
  
  # Number of points to sample (downsampling)
  num_points: 1024

  # [DP3 Only] Workspace bounds: [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
  workspace: [[-0.5, 0.5], [-0.5, 0.5], [0.0, 1.0]]
  
  # [DP3 Only] (Optional) Static rotation offset (Roll, Pitch, Yaw) applied to points before workspace cropping
  rpy: [0, 0, 0]

  # [IDP3 Only] Depth filtering bounds relative to camera
  near: 0.1
  far: 2.0

images:
  enabled: true  # If false, the 'image' subscriber is ignored
  
  # (Optional) Resize images before saving [width, height]. 
  # If omitted, saves at original resolution.
  resize: [84, 84]
```

