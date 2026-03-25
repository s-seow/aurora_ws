# Aurora Map Set-Sync Script

A ROS2 Python package (`py_srvcli`) for automating map loading and synchronization on a [SLAMTEC Aurora](https://www.slamtec.com/en/Aurora/Spec) device. The script connects to the Aurora unit, uploads a pre-built `.stcm` map file via the Slamware ROS2 SDK service, triggers a map sync, and then subscribes to live system status and robot pose topics.

Intended to be deployed on a Raspberry Pi as a persistent server, automating map loading and synchronization on an Aurora SLAMTEC device that uses 3D LiDAR to track stray buggies across airport grounds.

---

## How It Works

On execution, `map_set_sync_script`:

1. Launches the Slamware ROS2 SDK server and RViz as a subprocess (connects to Aurora at `192.168.11.1`).
2. Calls the `SyncSetStcm` service to upload the specified `.stcm` map file to the device.
3. Publishes a `SyncMapRequest` message to trigger a map sync.
4. Subscribes to `/system_status` and `/robot_pose`, logging pose data at 1 Hz until shutdown.

---

## Prerequisites

### 1. ROS2 Humble (Ubuntu 22.04)

Install following the [official ROS2 Humble development setup guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).

### 2. Slamware and Aurora SDKs

The package depends on `slamware_ros_sdk` and `aurora_remote_public`. Download both from the [SLAMTEC support page](https://www.slamtec.com/en/support#aurora) under **SDK and Firmware → ROS2 SDK**, then place both SDK folders into the `src/` directory.

SDK documentation: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/

### 3. Optional System Dependencies

Only required if using RViz or the CV bridge features:

```bash
sudo apt install -y libopencv-dev
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-rviz2
```

---

## Setup & Usage

**1. Install ROS2 dependencies**

```bash
rosdep install -i --from-paths src --rosdistro humble -y
```

**2. Build the workspace**

```bash
colcon build
```

**3. Source the overlay**

```bash
source install/setup.bash
```

**4. Run the script**

Place your `.stcm` map file in a `maps/` directory at the workspace root, then run:

```bash
ros2 run py_srvcli map_set_sync_script maps/<map_name>.stcm
```

> **Note:** The script internally sources `install/setup.bash`, `/opt/ros/humble/setup.bash`, and `~/aurora_ws/install/setup.bash` when launching the SDK subprocess. Ensure `aurora_ws` is built and sourced at that path, or adjust the `launch_cmd` in `map_set_sync_script.py` accordingly.

Launch logs are written to `/tmp/aurora_slamware_launch.log`.

---

## Miscellaneous Commands

| Purpose | Command |
|---|---|
| Launch RViz only | `ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1` |
| View system status | `ros2 topic echo /slamware_ros_sdk_server_node/system_status` |
| Load map manually | `ros2 service call /slamware_ros_sdk_server_node/sync_set_stcm slamware_ros_sdk/srv/SyncSetStcm "{'mapfile': 'maps/<map_name>.stcm'}"` |
| Sync map manually | `ros2 topic pub /slamware_ros_sdk_server_node/sync_map slamware_ros_sdk/msg/SyncMapRequest "{}" --once` |
| Debug SDK server | `ros2 run slamware_ros_sdk slamware_ros_sdk_server_node --ros-args -p ip_address:=192.168.11.1 --log-level debug` |

---

## Package Info

| Field | Value |
|---|---|
| Package | `py_srvcli` |
| ROS2 distro | Humble |
| Build type | `ament_python` |
| License | Apache 2.0 |
