# Aurora Script for Buggy Tracking

This repository contains a script for a SLAMTEC Aurora (https://www.slamtec.com/en/Aurora/Spec).
The functionalities include a) setting a map and b) syncing the map for the connected device at IP ``192.168.11.1``.

## Set-up 

### Dependencies

**ROS2**
```bash
rosdep install -i --from-paths src --rosdistro humble -y
colcon build
build  install  log  src
```

**Aurora/Slamware SDKs**

The script requires ``--slamware_ros_sdk`` and ``--aurora_remote_public`` to run.

Documentation found here: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/

Download from this page: https://www.slamtec.com/en/support#aurora (SDK and Firmware -> ROS2 SDK) and move into /src/

**Other Dependencies**

```bash
sudo apt install -y libopencv-dev
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-rviz2
```

### Steps

1. Build the workspace

    ``colcon build``

2. Run script with map_name selected in /maps/:

    ``ros2 run py_srvcli map_set_sync_script maps/(map_name).stcm``

## Individual misc. commands 

- **Run RVIZ separately**

    ``ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1``

- **View system status**

    ``ros2 topic echo /slamware_ros_sdk_server_node/system_status``

- **Load map manually**

    ``ros2 service call /slamware_ros_sdk_server_node/sync_set_stcm slamware_ros_sdk/srv/SyncSetStcm "{'mapfile': 'maps/(map_name).stcm'}"``

- **Sync map manually**

    ``ros2 topic pub /slamware_ros_sdk_server_node/sync_map slamware_ros_sdk/msg/SyncMapRequest "{}" --once``

- **Debug**

    ``ros2 run slamware_ros_sdk slamware_ros_sdk_server_node \ --ros-args -p ip_address:=192.168.11.1 --log-level debug``
