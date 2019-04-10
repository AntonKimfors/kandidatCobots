#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from agv_trans_comm.msg import ButtonPressed
from agv_trans_comm.msg import Command
from agv_trans_comm.msg import State


# Author: Marcus Lindvarn
class test_cc():
    def __init__ (self):
        global cc_state
        cc_state = Command()
        self.last_agv_cmd_recieved = ""
        self.last_agv_cmd_sent = ""
        self.last_run_sent = False
        self.last_state_recieved = ""
        self.last_product_sent = ""
        cc_state.command = ""
        cc_state.run = False
        cc_state.product_name = ""

        
        self.cc_state_pub = rospy.Publisher('/commandcenter', Command)
        #subscribing to the commandcenter topic as well as the button_states from the teleop.
        rospy.Subscriber("/AGV_state", State, self.callback_agvtest)
        rospy.Subscriber("/button_state", ButtonPressed, self.callback_button_state)
        # starts the node
        rospy.init_node('testcc_node')
        rospy.spin()

    # receive and update states when a message is published to /AGV_state
    def callback_agvtest(self, data):
        self.last_agv_cmd_recieved = data.cmd
        self.last_state_recieved = data.state
        #if recieved cmd is equal to send cmd, set run to True

        if (self.last_agv_cmd_sent == data.cmd):
            if (self.last_state_recieved = "init"):
                cc_state.run=True
        
        #always respond to a sent message at /AGV_state by responding to /commandcenter 
        self.cc_state_pub.publish(cc_state)
    
    def callback_button_state(self, data):
        #agv_state = State()
        # if A is pressed, send new mission
        if (data.apress == True):
            self.last_agv_cmd_sent = "Go to 0"
            cc_state.command = self.last_agv_cmd_sent
            cc_state.run = self.last_run_sent
            cc_state.product_name = "greg"
            self.cc_state_pub.publish(cc_state)
        # if B is pressed, set status to not running params and send
        if (data.ypress == True):
            cc_state.command = ""
            cc_state.run = False
            cc_state.product_name = ""
        #Publish with new changes
            self.cc_state_pub.publish(cc_state)

    def refresh_view(self):
        print('Last command recieved: ' + self.last_agv_cmd_recieved)
        print('Last command sent: ' + self.last_agv_cmd_sent)

    # Intializes everything
if __name__ == '__main__':
    try:
        test_cc()
        rospy.init_node('testCC')
    except rospy.ROSInterruptException:
        pass
