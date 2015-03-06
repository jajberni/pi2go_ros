__author__ = 'Jose Jimenez-Berni'

import RPi.GPIO as GPIO
import time


class SonarControl():

    def __init__(self):
        # Define Sonar Pin (same pin for both Ping and Echo
        self.gpio_sonar = 8


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
        return distance