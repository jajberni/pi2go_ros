#!/usr/bin/env python2.7

__author__ = 'Jose Jimenez-Berni'

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32


class SonarControl():

    def __init__(self):
        # Define Sonar Pin (same pin for both Ping and Echo
        self.gpio_sonar = 8
        rospy.init_node("sonar_control")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        self.rate = rospy.get_param('~rate', 5.0)  # the rate at which to publish

        self.pub_distance = rospy.Publisher('sonar_distance', Float32)

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.get_sonar_distance()
            r.sleep()

    def get_sonar_distance(self):
        GPIO.setup(self.gpio_sonar, GPIO.OUT)
        # Send 10us pulse to trigger
        GPIO.output(self.gpio_sonar, True)
        time.sleep(0.00001)
        GPIO.output(self.gpio_sonar, False)
        start = time.time()
        count = time.time()
        GPIO.setup(self.gpio_sonar, GPIO.IN)
        while GPIO.input(self.gpio_sonar) == 0 and time.time()-count < 0.1:
            start = time.time()
        count = time.time()
        stop = count
        while GPIO.input(self.gpio_sonar) == 1 and time.time()-count < 0.1:
            stop = time.time()
        # Calculate pulse length
        elapsed = stop-start
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound 34000(cm/s) divided by 2
        distance = elapsed * 17000
        self.pub_distance.publish(distance)

if __name__ == '__main__':
    """ main """
    sonar_control = SonarControl()
    sonar_control.spin()