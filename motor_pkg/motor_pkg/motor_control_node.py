
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
import RPi.GPIO as GPIO
from time import sleep
from dataclasses import dataclass
import threading    
import multiprocessing

FORWARD     = GPIO.HIGH
BACKWARD    = GPIO.LOW
UPWARD      = GPIO.HIGH
DOWNWARD    = GPIO.LOW
RIGHT       = 1
LEFT        = 0

MAX_X       = 9200
MIN_X       = 0

MAX_Z       = 30000
MIN_Z       = 0

limit_switch_x = 0
limit_switch_z = 0
touch_sensor = 0
laser_sensor = 0
push_button1 = 0
push_button2 = 0

@dataclass
class Motor:
    dir: int = 0
    pul: int = 0
    enable: bool = 0
    position: int = 0
    direction: bool = 0
    speed: int = 1000
    trigger: bool = 0
    initialized: bool = 0

class StepperMotorController(Node):
    def __init__(self):
        super().__init__('stepper_motor_controller')

        # Initialize motors with PUL and DIR pins
        self.xaxis_motor = Motor(dir=11, pul=9)
        self.yaxis_motor = Motor(dir=6,  pul=5)
        self.zaxis_motor = Motor(dir=26, pul=21)

        self.led_pin = 3
        self.pwm_pin = 12

        # Configurations
        self.speed = 1000
        self.delay = 1 / self.speed
        self.xaxis_motor.position = 0
        self.zaxis_motor.position = 0

        self.sensor_trigger = 0
        self.tolerance = 20
        self.once = False
        self.next = False
        self.prev = False

        self.slots = [3700, 6400, 8900]
        self.slotsR= [8900, 6400, 3700]


        # GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # GPIO setup
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.xaxis_motor.dir, GPIO.OUT)
        GPIO.setup(self.xaxis_motor.pul, GPIO.OUT)
        GPIO.setup(self.yaxis_motor.dir, GPIO.OUT)
        GPIO.setup(self.yaxis_motor.pul, GPIO.OUT)
        GPIO.setup(self.zaxis_motor.dir, GPIO.OUT)
        GPIO.setup(self.zaxis_motor.pul, GPIO.OUT)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)

        # ROS2 Subscriber to /sensors topic
        self.sensor_subscriber = self.create_subscription(
            UInt8,
            '/sensors',
            self.sensor_callback,
            10
        )

        self.xaxis_thread = threading.Thread(target=self.xaxis_motor_thread)
        self.yaxis_thread = threading.Thread(target=self.yaxis_motor_thread)
        self.zaxis_thread = threading.Thread(target=self.zaxis_motor_thread)

        self.main_th      = threading.Thread(target=self.main_thread)
        self.LEDs_th      = threading.Thread(target=self.leds_thread)

        self.xaxis_thread.start()
        self.zaxis_thread.start()
        self.yaxis_thread.start()

        self.main_th.start()
        self.LEDs_th.start()

    def leds_thread(self):

        while True:
            GPIO.output(self.led_pin, GPIO.HIGH)
            sleep(0.5)  # Keep the LED on for 1 second
            GPIO.output(self.led_pin, GPIO.LOW)
            sleep(0.5)  # Keep it off for 1 second

    def main_thread(self):

        global limit_switch_x
        global limit_switch_z
        global touch_sensor
        global laser_sensor
        global push_button1
        global push_button2

        while(not push_button1):
            sleep(0.1)

        while (1):

            while (not self.xaxis_motor.initialized) or (not self.zaxis_motor.initialized):
                self.initialize_motors()

            while (not laser_sensor) or not ((self.xaxis_motor.position > 800) or (self.xaxis_motor.position < 850)) :

                ''' or (not (self.zaxis_motor.position > 18000) or not (self.zaxis_motor.position < 18050))'''

                if (not (self.xaxis_motor.position > 810) or not (self.xaxis_motor.position < 840)):
                    self.run_motor_to_position(self.xaxis_motor, 5000, 820)
                    sleep(0.01)
                else:
                    self.stop_motor(self.xaxis_motor)

                if (not (self.zaxis_motor.position > 7000) or not (self.zaxis_motor.position < 7050)):
                    self.run_motor_to_position(self.zaxis_motor, 10000, 7020)
                else:
                    self.zaxis_motor.enable = 0

                self.xaxis_motor.enable = 0
                if not laser_sensor:
                    self.run_motor(self.yaxis_motor, 1, 2000)
                else:
                    self.yaxis_motor.enable = 0

                sleep(0.01)
                

            self.yaxis_motor.enable = 0
            self.yaxis_motor.position = 0
            # self.stop_motor(slef.yaxis_motor)

            while (not self.get_maxz()):
                sleep(0.01)

            self.sensor_trigger = 0
            # if not self.once:
            position = self.zaxis_motor.position

            # self.once = True


            while (not (self.zaxis_motor.position > (position - 100)) and self.once):
                self.run_motor_to_position(self.zaxis_motor, 5000, (position - 90))  
                sleep(0.01)     

            self.zaxis_motor.enable = 0

            for x in range(3):
                while not (self.slots[x] - self.tolerance <= self.xaxis_motor.position <= self.slots[x] + self.tolerance) or not (position - self.tolerance <= self.zaxis_motor.position <= position + self.tolerance):

                    if not x == 0:
                        if (not (self.zaxis_motor.position < (position - 500)) and not self.prev) :
                            self.run_motor_to_position(self.zaxis_motor, 2000, (position - 520))
                        else:
                            self.prev = True
                            self.next = True

                        if (not (self.zaxis_motor.position > position - 100)) and self.next:
                            self.run_motor_to_position(self.zaxis_motor, 2000, position - 80)

                        elif((self.zaxis_motor.position > position) and self.next):
                            self.stop_motor(self.zaxis_motor)

                    self.run_motor_to_position(self.xaxis_motor, 6000, self.slots[x] + 10)
                    sleep(0.01)

                self.get_logger().info(f"Position X: {self.xaxis_motor.position}")
                self.clean_motor()

                self.prev = False
                self.next = False

            self.zaxis_motor.enable = 0
            self.xaxis_motor.enable = 0
            self.yaxis_motor.position = 0

            while (not (self.yaxis_motor.position > 1500)):
                self.run_motor_to_position(self.yaxis_motor, 2000, 1510)
                # self.get_logger().info(f"Position y: {self.yaxis_motor.position}")
                sleep(0.01)

            self.yaxis_motor.enable = 0

            for x in range(3):
                while not (self.slotsR[x] - self.tolerance <= self.xaxis_motor.position <= self.slotsR[x] + self.tolerance) or not (position - self.tolerance <= self.zaxis_motor.position <= position + self.tolerance):

                    if not x == 0:
                        if (not (self.zaxis_motor.position < (position - 500)) and not self.prev) :
                            self.run_motor_to_position(self.zaxis_motor, 2000, (position - 580))
                        else:
                            self.prev = True
                            self.next = True

                        if (not (self.zaxis_motor.position > position - 100 )) and self.next:
                            self.run_motor_to_position(self.zaxis_motor, 2000, position - 80)

                        elif((self.zaxis_motor.position > position) and self.next):
                            self.stop_motor(self.zaxis_motor)

                    self.run_motor_to_position(self.xaxis_motor, 6000, self.slotsR[x] + 10)
                    sleep(0.01)

                self.get_logger().info(f"Position X: {self.xaxis_motor.position}")
                self.clean_motor()

                self.prev = False
                self.next = False

            self.zaxis_motor.enable = 0
            self.xaxis_motor.enable = 0


            self.run_motor(self.yaxis_motor, 1, 2000)
            self.run_motor(self.zaxis_motor, 1, 5000)

            sleep(2)

            self.yaxis_motor.enable = 0
            self.zaxis_motor.enable = 0

            # break
            sleep(0.01)

    def control_pwm(self, angle):
        duty_cycle = 2.5 + (angle / 18.0)
        self.pwm.ChangeDutyCycle(duty_cycle)
        # self.get_logger().info(f'Set servo angle to {angle} degrees')

    def xaxis_motor_thread(self):
        while 1:

            while self.xaxis_motor.enable == 1:

                if (((self.xaxis_motor.direction == LEFT) and (self.xaxis_motor.position < MAX_X)) 
                or ((self.xaxis_motor.direction == RIGHT) and (self.xaxis_motor.position > MIN_X))
                or (not (self.xaxis_motor.initialized))):

                    if self.xaxis_motor.direction:
                        GPIO.output(self.xaxis_motor.dir, GPIO.LOW)
                        self.xaxis_motor.position = self.xaxis_motor.position - 1 
                    else:
                        GPIO.output(self.xaxis_motor.dir, GPIO.HIGH)
                        self.xaxis_motor.position = self.xaxis_motor.position + 1 

                    GPIO.output(self.xaxis_motor.pul, GPIO.HIGH)
                    sleep(1 / self.xaxis_motor.speed)
                    GPIO.output(self.xaxis_motor.pul, GPIO.LOW)
                    sleep(1 / self.xaxis_motor.speed)

                # else :
                #     break

            sleep(0.01)

    def yaxis_motor_thread(self):
        while 1:

            while self.yaxis_motor.enable == 1:

                if self.yaxis_motor.direction:
                    GPIO.output(self.yaxis_motor.dir, GPIO.HIGH)
                    self.yaxis_motor.position = self.yaxis_motor.position - 1 
                else:
                    GPIO.output(self.yaxis_motor.dir, GPIO.LOW)
                    self.yaxis_motor.position = self.yaxis_motor.position + 1 

                GPIO.output(self.yaxis_motor.pul, GPIO.HIGH)
                sleep(1 / self.yaxis_motor.speed)
                GPIO.output(self.yaxis_motor.pul, GPIO.LOW)
                sleep(1 / self.yaxis_motor.speed)


            sleep(0.01)

    def zaxis_motor_thread(self):
        while 1:

            while self.zaxis_motor.enable == 1:

                if (((self.zaxis_motor.direction == DOWNWARD) and (self.zaxis_motor.position < MAX_Z)) 
                or ((self.zaxis_motor.direction == UPWARD) and (self.zaxis_motor.position > MIN_Z))
                or (not (self.zaxis_motor.initialized))):

                    if self.zaxis_motor.direction:
                        GPIO.output(self.zaxis_motor.dir, GPIO.HIGH)
                        self.zaxis_motor.position = self.zaxis_motor.position - 1 
                    else:
                        GPIO.output(self.zaxis_motor.dir, GPIO.LOW)
                        self.zaxis_motor.position = self.zaxis_motor.position + 1 

                    GPIO.output(self.zaxis_motor.pul, GPIO.HIGH)
                    sleep(1 / self.zaxis_motor.speed)
                    GPIO.output(self.zaxis_motor.pul, GPIO.LOW)
                    sleep(1 / self.zaxis_motor.speed)

                # else :
                #     break


            sleep(0.001)

    def run_motor(self, Motor, direction, speed):

        Motor.speed = speed
        Motor.direction = direction
        Motor.enable = 1

    def stop_motor(self, Motor):

        Motor.enable = 0

    def run_motor_to_position(self, Motor, speed, position):

        if ( position < Motor.position):
            Motor.direction = 1
        elif ( position > Motor.position ):
            Motor.direction = 0

        if ( not (position == Motor.position) ):
            Motor.speed = speed
            Motor.enable = 1
        elif ( position == Motor.position ):
            Motor.enable = 0

    def clean_motor(self):

        self.zaxis_motor.enable = 0

        front = self.xaxis_motor.position + 150
        back  = self.xaxis_motor.position - 150

        tolerance = 5

        for x in range(4):
            # Move to 'back' position
            while not (back - tolerance <= self.xaxis_motor.position <= back + tolerance):
                self.run_motor_to_position(self.xaxis_motor, 3000, back)
                sleep(0.01)

            # Move to 'front' position
            while not (front - tolerance <= self.xaxis_motor.position <= front + tolerance):
                self.run_motor_to_position(self.xaxis_motor, 3000, front)
                sleep(0.01)

    def get_maxz(self):

        global touch_sensor

        if self.zaxis_motor.initialized:
            self.run_motor(self.zaxis_motor, DOWNWARD, 10000)
            if (touch_sensor):
                MAX_Z = self.zaxis_motor.position
                self.get_logger().info(f"Max z value is : {self.zaxis_motor.position}")
                self.stop_motor(self.zaxis_motor)
                self.sensor_trigger = 1
        
        return self.sensor_trigger

    def initialize_motors(self):

        if not self.xaxis_motor.initialized:
            self.run_motor(self.xaxis_motor, RIGHT, 7000)

        if limit_switch_x == 1:
            self.stop_motor(self.xaxis_motor)
            self.xaxis_motor.position = 0
            self.xaxis_motor.initialized = 1

        if not self.zaxis_motor.initialized:
            self.run_motor(self.zaxis_motor, UPWARD, 10000)

        if limit_switch_z == 1:
            self.stop_motor(self.zaxis_motor)
            self.zaxis_motor.position = 0
            self.zaxis_motor.initialized = 1

        sleep(0.01)

    def sensor_callback(self, msg):

        sensor_data   = msg.data

        global limit_switch_x
        global limit_switch_z
        global touch_sensor
        global laser_sensor
        global push_button1
        global push_button2

        limit_switch_x = sensor_data & 0b00000001
        limit_switch_z = (sensor_data & 0b00000010) >> 1
        touch_sensor   = (sensor_data & 0b00000100) >> 2
        laser_sensor   = (sensor_data & 0b00001000) >> 3
        push_button1   = (sensor_data & 0b00010000) >> 4
        push_button2   = (sensor_data & 0b00100000) >> 5

    def destroy(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StepperMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Stepper Motor Controller Node')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
