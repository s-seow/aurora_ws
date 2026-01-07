import os

import sys
import subprocess
import rclpy
from rclpy.node import Node
import time

from slamware_ros_sdk.srv import SyncSetStcm
from slamware_ros_sdk.msg import SyncMapRequest, SystemStatus
from geometry_msgs.msg import PoseStamped as RobotPose


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")

        self.map_setter = self.create_client(
            SyncSetStcm, "/slamware_ros_sdk_server_node/sync_set_stcm"
        )
        while not self.map_setter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.setMap = SyncSetStcm.Request()
        self.map_publisher = self.create_publisher(
            SyncMapRequest, "/slamware_ros_sdk_server_node/sync_map", 10
        )

        self.reqMap = SyncMapRequest()
        self.subStatus = self.create_subscription(
            SystemStatus,
            "/slamware_ros_sdk_server_node/system_status",
            self.listener_callback_system_status,
            10,
        )

        self.subPose = self.create_subscription(
            RobotPose, "/slamware_ros_sdk_server_node/robot_pose", self.listener_callback_robot_pose, 10
        )

        self._last_pose_log_ts = 0.0

    def send_map(self, map_path):
        self.setMap.mapfile = map_path
        return self.map_setter.call_async(self.setMap)

    def send_request(self):
        msg = SyncMapRequest()
        self.map_publisher.publish(msg)

    def listener_callback_system_status(self, msg):
        #self.get_logger().info('System Status: "%s"' % msg.status)
        return
    
    def listener_callback_robot_pose(self, msg):
        now = time.time()
        if now - self._last_pose_log_ts < 1.0:
            return
        self._last_pose_log_ts = now

        self.get_logger().info(
            'Robot Pose: x="%f", y="%f", z="%f"'
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        )
        self.get_logger().info(
            'Robot Orientation: x="%f", y="%f", z="%f", w="%f"'
            % (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )
        )


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run py_srvcli map_set_sync_script maps/<map_name>.stcm")
        raise SystemExit(2)

    map_path = str(sys.argv[1])

    clean_env = {
        "HOME": os.environ.get("HOME"),
        "USER": os.environ.get("USER"),
        "PATH": "/usr/bin:/bin",
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
        "XDG_RUNTIME_DIR": os.environ.get("XDG_RUNTIME_DIR", ""),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
    }

    launch_log_path = "/tmp/aurora_slamware_launch.log"
    launch_log = open(launch_log_path, "w", buffering=1)

    launch_cmd = (
        "source install/setup.bash &&" # needed for run
        "source /opt/ros/humble/setup.bash && "
        "source ~/aurora_ws/install/setup.bash && "
        "export LD_LIBRARY_PATH=~/aurora_ws/src/aurora_remote_public/lib/linux_x86_64:$LD_LIBRARY_PATH && "
        "ros2 launch slamware_ros_sdk slamware_ros_sdk_server_and_view.xml ip_address:=192.168.11.1" # to launch rviz
    )

    slam_proc = subprocess.Popen(
        ["bash", "--noprofile", "--norc", "-lc", launch_cmd],
        env=clean_env, # Uncomment if clean environment needed
        stdout=launch_log,
        stderr=subprocess.STDOUT,
    )

    try:
        rclpy.init()
        minimal_client = MinimalClientAsync()

        if slam_proc.poll() is not None:
            raise RuntimeError(
                f"Slamware launch exited early (code={slam_proc.returncode}). "
                f"Check logs: {launch_log_path}"
            )

        future = minimal_client.send_map(map_path)
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(f"{response}")

        minimal_client.send_request()
        minimal_client.get_logger().info("Map Synced")

        rclpy.spin(minimal_client)

        minimal_client.destroy_node()
        rclpy.shutdown()

    finally:
        try:
            slam_proc.terminate()
            slam_proc.wait(timeout=10)
        except Exception:
            try:
                slam_proc.kill()
            except Exception:
                pass

        try:
            launch_log.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
