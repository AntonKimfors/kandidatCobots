#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16
from cth_hrp_cobot.msg import ButtonMsg

# Author: Marcus Lindvarn
# This ROS Node converts Joystick inputs from the joy node
# into commands for HRP
# Small changes has been made from the original file to make 
# it usable for this purpose


# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed

class xbox_teleop():   
    def __init__ (self):
        # publishing to "/cmd_vel" to control AGV
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)# test diff values for queue_size
        
        self.buttonpub = rospy.Publisher('/button_state', ButtonMsg, queue_size=1) # test diff values for queue_size
        self.linConst = 0.0
        self.angConst = 0.0
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, callback)
        # starts the node
        rospy.init_node('Joy2HRP')
        rospy.spin()

    def callback(data):
        self.twist = Twist()
        self.buttonpress = ButtonMsg()
        # vertical left stick axis = linear rate
        self.twist.linear.x = self.linConst*data.axes[1]
        # horizontal left stick axis = turn rate
        self.twist.angular.z = self.angConst*data.axes[0]
        
        
        self.buttonpress.xpress = False
        self.buttonpress.bpress = False
        self.buttonpress.apress = False  
        self.buttonpress.ypress = False 
        self.buttonpress.rbpress = False 
        self.buttonpress.lbpress = False  
        # A Pressed
        if (data.buttons[0]== 1):
            self.buttonpress.apress = True
            self.buttonpub.publish(self.buttonpress)
        # B Pressed
        if (data.buttons[1]== 1):
            self.buttonpress.bpress = True
            self.buttonpub.publish(self.buttonpress)
        # X Pressed
        if (data.buttons[2]== 1):
            self.buttonpress.xpress = True
            self.buttonpub.publish(self.buttonpress)
        # Y Pressed
        if (data.buttons[3]== 1):
            self.buttonpress.ypress = True
            self.buttonpub.publish(self.buttonpress)
        # RB Pressed - Unlock Movement
        if (data.buttons[4]== 1):
            self.linConst = 0.4
            self.angConst = 0.7
        # LB Pressed - Lock Movement
        if (data.buttons[5]== 1):
            self.linConst = 0.0
            self.angConst = 0.0
    
        pub.publish(twist)





if __name__ == '__main__':
    try:
        xbox_teleop()
    except rospy.ROSInterruptException:
        pass

