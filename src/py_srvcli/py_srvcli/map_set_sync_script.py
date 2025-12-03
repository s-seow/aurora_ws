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
        super().__init__('minimal_client_async')
        self.map_setter = self.create_client(SyncSetStcm, '/slamware_ros_sdk_server_node/sync_set_stcm')
        while not self.map_setter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.setMap = SyncSetStcm.Request()
        self.map_publisher = self.create_publisher(SyncMapRequest, '/slamware_ros_sdk_server_node/sync_map', 10)

        self.reqMap = SyncMapRequest()
        self.subStatus = self.create_subscription(SystemStatus, '/slamware_ros_sdk_server_node/system_status', self.listener_callback_system_status, 10)
        
        self.subPose = self.create_subscription(RobotPose, '/robot_pose', self.listener_callback_robot_pose, 10)

    def send_map(self, map_path): 
        self.setMap.mapfile = map_path
        return self.map_setter.call_async(self.setMap)
    
    def send_request(self):
        msg = SyncMapRequest()
        self.map_publisher.publish(msg)

    def listener_callback_system_status(self, msg):
        self.get_logger().info('System Status: "%s"' % msg.status)

    def listener_callback_robot_pose(self, msg):
        self.get_logger().info('Robot Pose: x="%f", y="%f", z="%f"' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.get_logger().info('Robot Orientation: x="%f", y="%f", z="%f", w="%f"' % (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        time.sleep(1)


def main():

    slam_proc = subprocess.Popen(
        [
        "ros2", "launch", "slamware_ros_sdk", "slamware_ros_sdk_server_and_view.xml", "ip_address:=192.168.11.1",
        ],

    # Following 2 lines silences rviz logs    
    stdout = subprocess.DEVNULL,
    stderr = subprocess.DEVNULL,
    )

    try:
        rclpy.init()
        
        minimal_client = MinimalClientAsync()
        
        future = minimal_client.send_map(str(sys.argv[1]))
        rclpy.spin_until_future_complete(minimal_client, future)
        response = future.result()
        minimal_client.get_logger().info(f"{response}")
        
        minimal_client.send_request()
        minimal_client.get_logger().info("Map Synced")

        # Uncomment spin to keep node alive 
        rclpy.spin(minimal_client)
        
        minimal_client.destroy_node()
        rclpy.shutdown()

    finally:
        slam_proc.terminate()
        slam_proc.wait()

if __name__ == '__main__':
    main()
