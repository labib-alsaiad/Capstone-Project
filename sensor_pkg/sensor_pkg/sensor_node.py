
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
import RPi.GPIO as GPIO
from time import sleep
import threading

class SensorAndPWMController(Node):
    def __init__(self):
        super().__init__('sensor_and_pwm_controller')
        
        # GPIO Pins
        self.limit_switchx_pin = 14
        self.limit_switchz_pin = 15
        self.touch_sensor_pin  = 7
        self.laser_sensor_pin  = 20
        self.push_button1_pin  = 24
        self.push_button2_pin  = 23
        self.pwm_pin = 12
        self.led_pin = 4

        self.sensor_value = 0
        self.touch_counter = 0

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.limit_switchx_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.limit_switchz_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.touch_sensor_pin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.laser_sensor_pin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.push_button1_pin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.push_button2_pin,  GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        GPIO.setup(self.led_pin, GPIO.OUT)

        # PWM setup
        self.pwm_frequency = 50  # 50 Hz for standard servo
        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_frequency)
        self.pwm.start(0)  # Start PWM with 0% duty cycle

        # ROS2 Publisher
        self.sensor_publisher = self.create_publisher(UInt8, '/sensors', 10)

        # Create a timer for periodic sensor checks and PWM control
        self.timer = self.create_timer(0.01, self.timer_callback)  # 50 Hz


        # Log initialization
        self.get_logger().info('Sensor and PWM Controller Node Initialized')

        self.reset_counter = 0
        self.control_pwm(180) 
        self.control_pwm(10) 
        self.touch_sensor_ready = 1

        self.sensor_th = threading.Thread(target=self.sensor_thread)
        self.LEDs_th = threading.Thread(target=self.leds_thread)

        self.sensor_th.start()
        self.LEDs_th.start()


    def control_pwm(self, angle):
        duty_cycle = 2.5 + (angle / 18.0)
        self.pwm.ChangeDutyCycle(duty_cycle)
        # self.get_logger().info(f'Set servo angle to {angle} degrees')

    def leds_thread(self):

        while True:
            GPIO.output(self.led_pin, GPIO.HIGH)
            sleep(0.5)  # Keep the LED on for 1 second
            GPIO.output(self.led_pin, GPIO.LOW)
            sleep(0.5)  # Keep it off for 1 second

    def sensor_thread(self):

        while 1:

            limit_switchx_state = GPIO.input(self.limit_switchx_pin) == 0  # Pressed = 0
            limit_switchz_state = GPIO.input(self.limit_switchz_pin) == 0  # Pressed = 0
            touch_sensor_state  = GPIO.input(self.touch_sensor_pin)  == 1  # Activated = 0
            laser_sensor_state  = GPIO.input(self.laser_sensor_pin)  == 0  # Activated = 0
            push_button1_state  = GPIO.input(self.push_button1_pin)  == 1  # Activated = 1
            push_button2_state  = GPIO.input(self.push_button2_pin)  == 1  # Activated = 1

            # Combine sensor states into a single uint8 message
            self.sensor_value = 0
            self.sensor_value |= (1 if limit_switchx_state else 0) << 0  # 1st bit
            self.sensor_value |= (1 if limit_switchz_state else 0) << 1  # 2nd bit
            self.sensor_value |= (1 if touch_sensor_state  else 0) << 2  # 3rd bit
            self.sensor_value |= (1 if laser_sensor_state  else 0) << 3  # 4th bit
            self.sensor_value |= (1 if push_button1_state  else 0) << 4  # 5rd bit
            self.sensor_value |= (1 if push_button2_state  else 0) << 5  # 6th bit

            if touch_sensor_state:
                sleep(0.1)

            if touch_sensor_state:
                self.control_pwm(90) 
                self.reset_counter += 1
                if self.reset_counter > 10:
                    self.control_pwm(180) 
                    self.get_logger().info('Reset touch sensor')
            else:
                self.reset_counter = 0

            if push_button2_state:
                self.control_pwm(180)
                self.control_pwm(10)

            # Log sensor states
            # self.get_logger().info(f'Sensor values: {bin(self.sensor_value)}')
            # self.get_logger().info(f'Touch sensor value: {(touch_sensor_state)}')

            sleep(0.005)

    def timer_callback(self):

        # Publish sensor values
        msg = UInt8()
        msg.data = self.sensor_value
        self.sensor_publisher.publish(msg)

    def destroy(self):
        # Clean up GPIO on shutdown
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorAndPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Sensor and PWM Controller Node')
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
