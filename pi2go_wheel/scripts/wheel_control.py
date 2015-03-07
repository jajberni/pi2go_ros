#!/usr/bin/env python2.7

__author__ = 'Jose A. Jimenez-Berni'

# Parts from original Pi2Go controller by Gareth Davies and Zachary Igielman, May 2014
# TODO: Wheel control using GPIO

# Import all necessary libraries
import RPi.GPIO as GPIO
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class WheelControl():

    def __init__(self):

        # Define Type of Pi2Go
        PGNone = 0
        PGFull = 1
        PGLite = 2
        # At the moment we only support PGLite
        self.PGType = PGLite

        rospy.init_node("wheel_control")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        self.w = rospy.get_param("~base_width", 0.2)

        self.pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32)
        self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32)

        rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.pwm_scale = rospy.get_param("~pwm_scale", 10)
        self.left = 0
        self.right = 0
        self.pwm_left_forward = None
        self.pwm_left_reverse = None
        self.pwm_right_forward = None
        self.pwm_right_reverse = None
        self.init_gpio()

    def init_gpio(self):
        # Disable warnings
        GPIO.setwarnings(False)

        # Pins 24, 26 Left Motor
        # Pins 19, 21 Right Motor
        gpio_left_forward = 26
        gpio_left_reverse = 24
        gpio_right_forward = 19
        gpio_right_reverse = 21

        # Define obstacle sensors and line sensors
        ir_front_left = 7
        ir_front_right = 11

        line_right = 13
        line_left = 12

        # Define GPIO pins for Front/rear LEDs on Pi2Go-Lite
        gpio_front_led = 15
        gpio_back_led = 16

        # Define Sonar Pin (same pin for both Ping and Echo
        self.gpio_sonar = 8

        # Define pins for switch (different on each version)
        self.gpio_switch = 23

        # Define if servo background process is active
        self.servos_active = False

        rospy.loginfo("Setting up GPIOs")

        #use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        #set up digital line detectors as inputs
        GPIO.setup(line_right, GPIO.IN) # Right line sensor
        GPIO.setup(line_left, GPIO.IN) # Left line sensor

        #Set up IR obstacle sensors as inputs
        GPIO.setup(ir_front_left, GPIO.IN) # Left obstacle sensor
        GPIO.setup(ir_front_right, GPIO.IN) # Right obstacle sensor

        #use pwm on inputs so motors don't go too fast
        GPIO.setup(gpio_left_forward, GPIO.OUT)
        self.pwm_left_forward = GPIO.PWM(gpio_left_forward, 20)
        self.pwm_left_forward.start(0)

        GPIO.setup(gpio_left_reverse, GPIO.OUT)
        self.pwm_left_reverse = GPIO.PWM(gpio_left_reverse, 20)
        self.pwm_left_reverse.start(0)

        GPIO.setup(gpio_right_forward, GPIO.OUT)
        self.pwm_right_forward = GPIO.PWM(gpio_right_forward, 20)
        self.pwm_right_forward.start(0)

        GPIO.setup(gpio_right_reverse, GPIO.OUT)
        self.pwm_right_reverse = GPIO.PWM(gpio_right_reverse, 20)
        self.pwm_right_reverse.start(0)

        GPIO.setup(gpio_front_led, GPIO.OUT)
        GPIO.setup(gpio_back_led, GPIO.OUT)

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():

            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()


    def spinOnce(self):
        # dx = (l + r) / 2
        # dr = (r - l) / w

        self.right = 1.0 * self.dx + self.dr * self.w / 2.0
        self.left = 1.0 * self.dx - self.dr * self.w / 2.0
        #rospy.loginfo("publishing: ({left}, {right})".format(left=self.left, right=self.right))

        self.pub_lmotor.publish(self.left)
        self.pub_rmotor.publish(self.right)

        #TODO: Set the actual PWM to the wheels
        self.go(self.left*self.pwm_scale, self.right*self.pwm_scale)

        self.ticks_since_target += 1


    def twistCallback(self, msg):
        #rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

    def getSwitch(self):
        val = GPIO.input(self.gpio_switch)
        return val == 0

    # go(leftSpeed, rightSpeed): controls motors in both directions independently using different
    # positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
    def go(self, left_speed, right_speed):
        left_speed = max(min(100, left_speed), -100)
        right_speed = max(min(100, right_speed), -100)

        if left_speed < 0:
            self.pwm_left_forward.ChangeDutyCycle(0)
            self.pwm_left_reverse.ChangeDutyCycle(abs(left_speed))
            self.pwm_left_reverse.ChangeFrequency(abs(left_speed) + 5)
        else:
            self.pwm_left_reverse.ChangeDutyCycle(0)
            self.pwm_left_forward.ChangeDutyCycle(left_speed)
            self.pwm_left_forward.ChangeFrequency(left_speed + 5)
        if right_speed < 0:
            self.pwm_right_forward.ChangeDutyCycle(0)
            self.pwm_right_reverse.ChangeDutyCycle(abs(right_speed))
            self.pwm_right_reverse.ChangeFrequency(abs(right_speed) + 5)
        else:
            self.pwm_right_reverse.ChangeDutyCycle(0)
            self.pwm_right_forward.ChangeDutyCycle(right_speed)
            self.pwm_right_forward.ChangeFrequency(right_speed + 5)


if __name__ == '__main__':
    """ main """
    wheel_control = WheelControl()
    wheel_control.spin()