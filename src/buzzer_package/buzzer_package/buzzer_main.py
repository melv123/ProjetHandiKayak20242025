import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from time import sleep
import RPi.GPIO as GPIO


class BuzzerNode(Node):
    """
    Node that subscribe on the topic 'buzzer_instruction' to get instructions about the frequency of the bip
    to make the buzzer buzz
    """

    def __init__(self):
        super().__init__('Buzzer_command')

        self.max_period = 2  # Maximum period between two bip in seconds
        self.min_period = 0.2  # Minimum period
        self.buzzer_frequencies = [400, 10000]  # Frequency of the buzzer in Hz [turn left, turn right]
        self.current_buzzer = 0

        self.buzzer_pin = 4  # GPIO 4, pin 7
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.setwarnings(False)
        self.buzzer = GPIO.PWM(self.buzzer_pin, 0.1)
        self.buzzer.start(50)
        self.buzzer.ChangeDutyCycle(0)  # Shut the buzzer down
            
        self.timer = self.create_timer(1, self.timer_callback)
        self.timer.cancel()
        self.subscription = self.create_subscription(Float32, 'buzzer_instruction', self.instruction_callback, 10)

        self.get_logger().info("BuzzerNode initialization finished. Listening on topic 'buzzer_instruction'.")
        # To publish on a terminal: ros2 topic pub -1 /buzzer_instruction std_msgs/msg/Float32 "{data: -0.4}"

    def buzz(self, frequency, duty_cycle=80.0, duration=0.2):
        """
        Activate the buzzer (the buzzer must be a passive buzzer)
        :param frequency: frequency of the buzzer in Hz
        :param duty_cycle: duty cycle of the pwm in %
        :param duration: duration of the buzz in seconds
        :return: None
        """
    
        self.buzzer.ChangeFrequency(frequency)
        self.buzzer.ChangeDutyCycle(duty_cycle)
        sleep(duration)
        self.buzzer.ChangeDutyCycle(0)  # Shut the buzzer down

    def timer_callback(self):
        """
        Callback function to make the buzzer buzz
        :return: None
        """
        #self.get_logger().info(f"Buzzer frequency: {self.buzzer_frequencies[self.current_buzzer]} Hz")
        self.buzz(self.buzzer_frequencies[self.current_buzzer])

    def instruction_callback(self, value):
        """
        Callback function that is activated when a message is published on the topic 'buzzer_instruction'.
        The value must be between -1 and 1 and is a Float32
        :param value: Float32: value of the message on the topic, must be between -1 and 1
        :return: None
        """
        #self.get_logger().info(f"Received instruction: {value.data}")

        if not self.timer.is_canceled():
            self.timer.cancel()  # Stop the timer to change its period

        if value.data == 0:
            return  # No need to relaunch the timer

        data = value.data
        if data < 0:
            data = -data
            self.current_buzzer = 1  # Buzzer to rotate right
        else:
            self.current_buzzer = 0  # Buzzer to rotate left

        # Set the period of the timer
        self.timer = self.create_timer((self.min_period - self.max_period) * data + self.max_period,
                                       self.timer_callback)
        #self.get_logger().info(f"Received instruction: {value.data}")


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
