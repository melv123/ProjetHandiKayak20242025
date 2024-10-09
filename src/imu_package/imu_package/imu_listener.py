import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from icm20948 import ICM20948
import struct

class MyNode(Node):
    """
    __init__ initialises the global processes and variables
    """

    def __init__(self):
        super().__init__('imu_listener')
        self.period = 0.028  # Period between callbacksb not actually followed due to the second callback actuall period is 0.01
       
        # Create the publisher of an IMU message on the node Imu_readings
        self.publisher_gyro = self.create_publisher(Float32MultiArray, 'imu/gyroscope', 10)
        self.publisher_acc = self.create_publisher(Float32MultiArray, 'imu/accelerometer', 10)
        self.publisher_mag = self.create_publisher(Float32MultiArray, 'imu/magnetometer', 10)
        
        # Launches the function timer_callbacks every period
        self.timer_ = self.create_timer(self.period,  self.timer_callbacks)
        self.get_logger().info('Node initialised')
        self.sensor = ICM20948()  # Initialise the IMU ICM20948
        


    """
    timer_callbacks takes the imu data and publishes it on the node Imu_readings
    """

    def timer_callbacks(self):
        # Get the IMU data
        acc_gyro_measurement = self.sensor.read_accelerometer_gyro_data()
        acc_measurement = acc_gyro_measurement[0:3]  # acc in g
        gyro_measurement = acc_gyro_measurement[3:]  # gyro in degree per second
        mag_measurement = self.read_magnetometer_data()

        #Initialize the variables
        acc = Float32MultiArray()
        gyro = Float32MultiArray()
        mag = Float32MultiArray()

        #Save the data
        acc.data = acc_measurement
        gyro.data = gyro_measurement 
        mag.data = mag_measurement

        #Publish
        self.publisher_acc.publish(acc)
        self.publisher_gyro.publish(gyro)
        self.publisher_mag.publish(mag)    

    def read_magnetometer_data(self, timeout=1.0):
        AK09916_CNTL2 = 0x31
        AK09916_HXL = 0x11
        self.sensor.mag_write(AK09916_CNTL2, 0x01)  # Trigger single measurement

        data = self.sensor.mag_read_bytes(AK09916_HXL,6)

        # Read ST2 to confirm self.read finished,
        # needed for continuous modes
        # self.mag_read(AK09916_ST2)

        x, y, z = struct.unpack("<hhh", bytearray(data))

        # Scale for magnetic flux density "uT"
        # from section 3.3 of the datasheet
        # This value is constant
        x *= 0.15
        y *= 0.15
        z *= 0.15

        return x, y, z

    """
    timer_callbacks takes the imu data and publishes it on the node Imu_readings
    """

    def timer_callbacks_mag(self):
        # Get the IMU data
        mag_measurement = self.read_magnetometer_data2()
        
        mag = Float32MultiArray()

        mag.data = mag_measurement
        
        self.publisher_mag.publish(mag)  
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    sensor.use_I2C()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
