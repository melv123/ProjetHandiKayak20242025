import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import numpy as np
from .submodules.kayak_function import KayakFunctions
from tf_transformations import euler_from_quaternion


class MyNode(Node):
    """
    __init__ initialises the global processes and variables
    """

    def __init__(self):
        super().__init__('ekf_listener')
        self.period = 0.5  # Period between callbacks
        # Create the publisher for the Float32 buzzer instructions on the topic buzzer_instruction
        self.publisher_ = self.create_publisher(Float32, 'buzzer_instruction', 10)

        # Subscribes to the node odometry/filtered and call the callback function
        # once a Odometry data is published on it
        self.subscribtion_ = self.create_subscription(Odometry, 'odometry/filtered', self.callback, 10)
        
        # Call the timer_callbacks function once a period
        self.timer_ = self.create_timer(self.period, self.timer_callbacks)
        self.get_logger().info('Node initialised')
        self.position = Vector3()
        self.euler = Vector3()

    """
    quat_2_euler takes a quaternion and returns a Vector3 with the euler angles
    @param array3 a numpy array of size 3
    @return vect3 an object of type Vector3
    """

    @staticmethod
    def quat_2_euler(quat: Quaternion) -> Vector3:
        euler = Vector3()
        # Euler angles (yaw, pitch and roll)
        euler.x, euler.y, euler.z = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler

    """
    timer_callbacks takes the position and orientation and determine the command to publish
    """

    def timer_callbacks(self):
        functions = KayakFunctions()
        buzzer_command = Float32()
        buzzer_command.data = functions.getOrder(self.euler.z, self.position.y, 0.4, 20., 10.)
        #self.get_logger().info(f"Calculated command : {buzzer_command.data}")
        #self.get_logger().info(f"Received angle: roll {self.euler.x},pitch {self.euler.y},yaw {self.euler.z}")

        self.publisher_.publish(buzzer_command)

    """
    callback takes the Odometry message and extract the position and euler angles from it
    """

    def callback(self, msg):
        self.position.y = msg.pose.pose.position.y
        self.euler = self.quat_2_euler(msg.pose.pose.orientation)

        #self.get_logger().info(f"Received quaternion: {msg.pose.pose.orientation}")
        #self.get_logger().info(f"Received angle: roll {self.euler.x},pitch {self.euler.y},yaw {self.euler.z}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
