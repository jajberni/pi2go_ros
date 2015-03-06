#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

pub = rospy.Publisher('/cmd_vel', Twist)

def callback(msg):
  cmd_msg = Twist()
  cmd_msg.linear.x = speed_factor * msg.axes[1]
  cmd_msg.angular.z = speed_factor * msg.axes[2]
  pub.publish(cmd_msg)
 
def joy_teleop():
  rospy.init_node('joy_teleop')
  # Get parameters from the server
  global speed_factor
  speed_factor = rospy.get_param('~speed_factor', 10.0)
  rospy.loginfo('Using speed_factor: [%.1f]' % speed_factor)
  # Susbscribe to the topic that contains the ps3 keys
  rospy.Subscriber('/joy', Joy, callback)   
  # Keeps python from exiting until this node is stopped
  rospy.spin()
 
if __name__ == '__main__':
  joy_teleop()
