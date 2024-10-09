import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import numpy as np
from tf_transformations import euler_from_quaternion
from .submodules.madgwick import Madgwick
from std_msgs.msg import Float32MultiArray

class MyNode(Node):
    
    """
    __init__ initialises the global processes and variables
    """
    def __init__(self):
        super().__init__('imu_treatement')
        self.period = 1/35  # Period between callbacks
        # Create the publisher of an IMU message on the node Imu_readings
        self.publisher_ = self.create_publisher(Imu, 'Imu_readings', 10)

        # Create the subscriber to the components of the imu data 
        self.subscribtion_gyro = self.create_subscription(Float32MultiArray, 'imu/gyroscope', self.callback_gyro, 10)
        self.subscribtion_acc = self.create_subscription(Float32MultiArray, 'imu/accelerometer', self.callback_acc, 10)
        self.subscribtion_mag = self.create_subscription(Float32MultiArray, 'imu/magnetometer', self.callback_mag, 10)
        
        # Launches the function timer_callbacks every period
        self.timer_ = self.create_timer(self.period,  self.timer_callbacks)

        #Define global elements
        self.madgwick = Madgwick()  # Initialise a Madgwick filter
        self.Q = np.array([1.0, 0.0, 0.0, 0.0])  # Initialise a quaternion
        self.acc_measurement = [0.,0.,1.]
        self.mag_measurement = [-5.,7.,2.]
        self.gyro_measurement = [-1.,1.,0.]
        self.get_logger().info('Node initialised')
        

    """
    ned_to_enu takes a list in the NED (North, East, Down) format and switch it to the ENU (East, North, Up) format
    @param ned a list of coordinates in the NED format
    @return enu a list of coordinates in the ENU format
    """
    @staticmethod
    def ned_to_enu(ned) -> tuple:
        return ned[1], ned[0], -ned[2]

    """
    imu_treatement gets the data from the IMU, traet it and returns it in the IMU format
    @return imu the IMU message
    """
    def imu_treatement(self):
        # Initialise variables
        imu = Imu()

        # Get time
        imu.header.stamp = self.get_clock().now().to_msg()

        # Assign the imu data to a vect3 object

        vec3_acc = self.assign_2_vect( self.ned_to_enu( self.acc_measurement ))
        vec3_gyro = self.assign_2_vect(self.ned_to_enu( self.gyro_measurement ))

        # Determine the quaternarion

        self.Q = self.madgwick.madgwick_ahrs_update(self.gyro_measurement[0], self.gyro_measurement[1], self.gyro_measurement[2],
                                                    self.acc_measurement[0], self.acc_measurement[1], self.acc_measurement[2],
                                                    self.mag_measurement[0], self.mag_measurement[1], self.mag_measurement[2], 1 / self.period)


        
        #file = open("orientation", "a")
        #file.writelines([str(self.Q[0]),",", str(self.Q[1]),",", str(self.Q[2]),",", str(self.Q[3]),"\n"])
        #file.close()

        # Checks the variables
        self.quaternion_Hamilton_2_JPL()

        # self.get_logger().info(f"Received acceleration: {vec3_acc}")
        # self.get_logger().info(f"Received gyro: {vec3_gyro}")
        # self.get_logger().info(f"Received mag: {mag_measurement}\n")
        # self.get_logger().info(f"Received quat imu: {self.Q}\n")
        # self.get_logger().info(f"Received euler: {self.quat_2_euler(self.assign_2_quat())}\n")

        # Assign the value to imu

        

        imu.linear_acceleration = vec3_acc
        
        imu.linear_acceleration_covariance = [5.548E-4, 0.0, 0.0,
                                              0.0, 5.065E-4, 0.0,
                                              0.0, 0.0, 5.804E-4]
        
        imu.angular_velocity = vec3_gyro
        
        imu.angular_velocity_covariance = [4.5598E-6, 0., 0.,
                                           0., 4.1822E-6, 0.,
                                           0., 0., 4.4396E-6]
        
        imu.orientation = self.assign_2_quat()

        # As the first term of the matrix is -1 the quaternion is currently ignored
        imu.orientation_covariance = [0.0001, 0., 0.,
                                      0., 0.0001, 0.,
                                      0., 0., 0.0001]

        return imu

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
    assign_2_vect takes a numpy array and returns a Vector3
    @param array3 a numpy array of size 3
    @return vect3 an object of type Vector3
    """
    @staticmethod
    def assign_2_vect(array3: np.array) -> Vector3:
        vect3 = Vector3()
        vect3.x, vect3.y, vect3.z = array3[0], array3[1], array3[2]

        return vect3

    """
    assign_2_quat assign Q to a Quaternion
    @return quat an object of type Quaternion
    """
    def assign_2_quat(self) -> Quaternion:
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = self.Q[0], self.Q[1], self.Q[2], self.Q[3]

        return quat

    """
    quaternion_Hamilton_2_JPL transforms a quater,io, from the Hamilton convention (w,x,y,z) tp JPL (x,y,z,w)
    """
    def quaternion_Hamilton_2_JPL(self):
        self.Q[0], self.Q[1], self.Q[2], self.Q[3] = self.Q[1], self.Q[2], self.Q[3], self.Q[0]

    """
    timer_callbacks takes the imu data and publishes it on the node Imu_readings
    """
    def timer_callbacks(self):
        self.publisher_.publish(self.imu_treatement())

    
    """
    callback_acc get the message sent on the imu/accelerometer topic and change it to S.I format
    """
    def callback_acc(self, msg):
        self.acc_measurement = self.ned_to_enu(msg.data)

        self.acc_measurement = [self.acc_measurement[0] * 9.81, self.acc_measurement[1] * 9.81,
                           self.acc_measurement[2] * 9.81]  # acc from g to m/(s*s)
    """
    callback_gyro get the message sent on the imu/gyroscope topic and change it to S.I format
    """
    def callback_gyro(self, msg):
        self.gyro_measurement = self.ned_to_enu(msg.data)

        self.gyro_measurement = [self.gyro_measurement[0] * np.pi / 180, self.gyro_measurement[1] * np.pi / 180,
                            self.gyro_measurement[2] * np.pi / 180]  # gyro from dps to radian/second
        
    """
    callback_mag get the message sent on the imu/magnetometer topic and change it to S.I format
    """
    def callback_mag(self, msg):
        self.mag_measurement = self.ned_to_enu(msg.data)

        self.mag_measurement = [self.mag_measurement[0] * 1E+3, self.mag_measurement[1] * 1E+3,
                           self.mag_measurement[2] * 1E+3]  # mag from microtesla to nanotesla


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    sensor.use_I2C()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
