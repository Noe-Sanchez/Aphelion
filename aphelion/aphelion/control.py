import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import time

class EnablePulseNode(Node):
    def __init__(self):
        super().__init__('enable_pulse_node')

        self.pin = 13  # BOARD 13
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)

        self.subscription = self.create_subscription(
            Bool,
            'enable_pulse',
            self.pulse_callback,
            10
        )

        self.pulse_duration = 0.3  # tiempo impulso
        self.get_logger().info('EnablePulseNode ready.')

    def pulse_callback(self, msg):
        if msg.data:
            self.get_logger().info('Received trigger, sending HIGH pulse.')
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(self.pulse_duration)
            GPIO.output(self.pin, GPIO.LOW)
            self.get_logger().info('Pulse complete.')
        else:
            self.get_logger().info('Received False — no pulse sent.')

    def destroy_node(self):
        GPIO.output(self.pin, GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EnablePulseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
