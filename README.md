# SLAMTEC Aurora Script for Buggy Tracking

This contains a script for a SLAMTEC Aurora (https://www.slamtec.com/en/Aurora/Spec) that allows for 1. setting a map and 2. syncing the map for the connected device at   IP ``192.168.11.1``.

## Setup ROS2 / Install Dependencies

``rosdep install -i --from-path src --rosdistro humble -y``....000000000000000000000000000000000000000000000....................

``colcon build``

``build  install  log  src``

## To run script

Build

``colcon build``

Source the overlay

``source install/setup.bash``

Export path

``export LD_LIBRARY_PATH=~/aurora_ws/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH``

Run script with map_name selected in /maps

``ros2 run py_srvcli map_set_sync_script maps/(map_name).stcm``

## To run RVIZ

``colcon build``

``source install/setup.bash``

``export LD_LIBRARY_PATH=~/aurora_ws/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH``

``ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1``

## Other commands after source+export

### see system status

``ros2 topic echo /slamware_ros_sdk_server_node/system_status``

### load map manually

``ros2 service call /slamware_ros_sdk_server_node/sync_set_stcm slamware_ros_sdk/srv/SyncSetStcm "{'mapfile': 'maps/(map_name).stcm'}"``

### sync map manually

``ros2 topic pub /slamware_ros_sdk_server_node/sync_map slamware_ros_sdk/msg/SyncMapRequest "{}" --once``

### debug

``ros2 run slamware_ros_sdk slamware_ros_sdk_server_node \ --ros-args -p ip_address:=192.168.11.1 --log-level debug``
