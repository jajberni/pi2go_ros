__author__ = 'Jose A. Jimenez-Berni'

# Parts from original Pi2Go controller by Gareth Davies and Zachary Igielman, May 2014
# TODO: Wheel control using GPIO

# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os
from Adafruit_PWM_Servo_Driver import PWM
#from sgh_PCF8591P import sgh_PCF8591P

# Define Type of Pi2Go
PGNone = 0
PGFull = 1
PGLite = 2
PGType = PGNone # Set to None until we find out which during init()

# Pins 24, 26 Left Motor
# Pins 19, 21 Right Motor
L1 = 26
L2 = 24
R1 = 19
R2 = 21

# Define obstacle sensors and line sensors
irFL = 7
irFR = 11
irMID = 15  # this sensor not available on Lite version
lineRight = 13
lineLeft = 12

# Define Colour IDs for the RGB LEDs (Pi2Go full only)
Blue = 0
Green = 1
Red = 2
pwmMax = 4095 # maximum PWM value

# Define GPIO pins for Front/rear LEDs on Pi2Go-Lite
frontLED = 15
rearLED = 16

# Define Sonar Pin (same pin for both Ping and Echo
sonar = 8

# Define pins for switch (different on each version)
switch = 16
Lswitch = 23

# Define if servo background process is active
ServosActive = False
