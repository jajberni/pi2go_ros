#! /usr/bin/env python
import rospy, math
import numpy as np
import sys, termios, tty, select, os
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class KeyTeleop(object):
  cmd_bindings = {'q':np.array([1,1]),
                  'w':np.array([1,0]),
                  'e':np.array([1,-1]),
                  'a':np.array([0,1]),
                  'd':np.array([0,-1]),
                  'z':np.array([-1,-1]),
                  'x':np.array([-1,0]),
                  'c':np.array([-1,1])
                  }
  set_bindings = { 't':np.array([1,1]),
                  'b':np.array([-1,-1]),
                  'y':np.array([1,0]),
                  'n':np.array([-1,0]),
                  'u':np.array([0,1]),
                  'm':np.array([0,-1])
                }
  def init(self):
    # Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0.5, 1.0])
    self.command = np.array([0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    self.using_teleop = False
    # Setup publishers
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist)
    self.pub_using_telop = rospy.Publisher('/using_telop', Bool)
    # Start the timer that will publish the teleop status at 10 Hz. Rate hard code on propose
    self.timer = rospy.Timer(rospy.Duration(1.0/10.0), self.using_teleop_timer)    
          
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    # Stop the timer
    
  def using_teleop_timer(self, event):
    if not rospy.is_shutdown():
      self.pub_using_telop.publish(Bool(self.using_teleop))
      
  def run(self):
    try:
      self.init()
      self.print_usage()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()

  def print_usage(self):
    msg = """
    Keyboard Teleop that Publish to /cmd_vel (geometry_msgs/Twist)
    Copyright (C) 2013
    Released under BSD License
    --------------------------------------------------
    H:       Print this menu
    Moving around:
      Q   W   E
      A   S   D
      Z   X   Z
    T/B :   increase/decrease max speeds 10%
    Y/N :   increase/decrease only linear speed 10%
    U/M :   increase/decrease only angular speed 10%
    anything else : stop
    
    G :   Quit
    --------------------------------------------------
    """
    self.loginfo(msg)
    self.show_status()
  
  # Used to print items to screen, while terminal is in funky mode
  def loginfo(self, str):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    print(str)
    tty.setraw(sys.stdin.fileno())
  
  # Used to print teleop status
  def show_status(self):
    msg = 'Status:\tlinear %.2f\tangular %.2f' % (self.speed[0],self.speed[1])
    self.loginfo(msg)
  
  # For everything that can't be a binding, use if/elif instead
  def process_key(self, ch):
    if ch == 'h':
      self.print_usage()
    elif ch in self.cmd_bindings.keys():
      self.command = self.cmd_bindings[ch]
      self.update()
      self.using_teleop = True
    elif ch in self.set_bindings.keys():
      self.speed = self.speed * (1 + self.set_bindings[ch]*self.inc_ratio)
      self.update()
      self.show_status()
    elif ch == 'g':
      self.loginfo('Quitting')
      # Stop the robot
      twist = Twist()
      self.pub_twist.publish(twist)
      self.timer.shutdown()
      rospy.sleep(0.5)
      rospy.signal_shutdown('Shutdown')
    else:
      self.using_teleop = False
      self.command = np.array([0, 0])
      self.update()
  
  def update(self):
    if rospy.is_shutdown():
      return
    twist = Twist()
    cmd  = self.speed*self.command
    twist.linear.x = cmd[0]
    twist.angular.z = cmd[1]
    self.pub_twist.publish(twist)
  
  # Get input from the terminal
  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key.lower()
 
if __name__ == '__main__':
  rospy.init_node('keyboard_teleop')
  teleop = KeyTeleop()
  teleop.run()
