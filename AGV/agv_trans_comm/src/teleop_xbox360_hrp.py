#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from agv_trans_comm.msg import ButtonPressed

# Author: Marcus Lindvarn
# This ROS Node converts Joystick inputs from the joy node
# into commands for HRP
# Small changes has been made from the original file to make 
# it usable for this purpose

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()
    buttonpress = ButtonPressed()
    # vertical left stick axis = linear rate
    twist.linear.x = 0.4*data.axes[1]
    # horizontal left stick axis = turn rate
    twist.angular.z = 0.7*data.axes[0]
    
    
    buttonpress.xpress = False
    buttonpress.bpress = False
    buttonpress.apress = False  # TESTING
    buttonpress.ypress = False  # TESTING
    if (data.buttons[9]== 1):
        print('stoppressed')
        twist.linear.x = 0*data.axes[1]
        twist.angular.z = 0*data.axes[0]
    if (data.buttons[2]== 1):
        print('Xpressed')
        buttonpress.xpress = True
        buttonpub.publish(buttonpress)  
    if (data.buttons[1]== 1):
        print('Bpressed')
        buttonpress.bpress = True
        buttonpub.publish(buttonpress)
    # ----------- START TESTING ----------------
    if (data.buttons[0]== 1):
        print('Apressed')
        buttonpress.apress = True
        buttonpub.publish(buttonpress)  
    if (data.buttons[3]== 1):
        print('Ypressed')
        buttonpress.ypress = True
        buttonpub.publish(buttonpress)
    # ----------- END TESTING ----------------
    
    pub.publish(twist)


# Intializes everything
def start():
    # publishing to "/cmd_vel" to control AGV
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist)
    global buttonpub
    buttonpub = rospy.Publisher('/button_state', ButtonPressed)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2HRP')
    rospy.spin()

if __name__ == '__main__':
    start()

