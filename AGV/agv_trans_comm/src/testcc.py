#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from agv_trans_comm.msg import ButtonPressed
from agv_trans_comm.msg import Command
from agv_trans_comm.msg import State


# Author: Marcus Lindvarn
'''
string command
bool run
string product_name
'''
def callback_agvtest(data):
    #if (last_agv_cmd_sent == data.cmd):
    #    cc_state.run=True
    
    cc_state_pub.publish(cc_state)

def callback_button_state(data):
    #agv_state = State()
    # if X is pressed, set status and send
    #print ("xpressed: " + data.xpress + '\n')
    #print ("bpressed: " + data.bpress + '\n')
    # if A is pressed, give new mission
    if (data.apress == True):
        last_agv_cmd_sent = "go to 0"
        cc_state.command = last_agv_cmd_sent
        cc_state.run = True
        cc_state.product_name = "greg"

    # if B is pressed, set status to not running params and send
    if (data.ypress == True):
        cc_state.command = ""
        cc_state.run = False
        cc_state.product_name = ""
    #Publish with new changes
    cc_state_pub.publish(cc_state)

def refresh_view(self):
    print('Last command: ' + last_command_recieved)

# Intializes everything
def start():    
    global cc_state
    cc_state = Command()
    #__init__(self)
    global last_agv_cmd_recieved
    global last_agv_cmd_sent
    global last_run_sent
    global last_state_recieved
    global last_product_sent

    cc_state.command = ""
    cc_state.run = False
    cc_state.product_name = ""


    cc_state_pub = rospy.Publisher('/commandcenter', Command)
    #subscribing to the comandcenter topic as well as the buttionstates from the teleop.
    rospy.Subscriber("/AGV_state", State, callback_agvtest)
    rospy.Subscriber("/button_state", ButtonPressed, callback_button_state)
    # starts the node
    rospy.init_node('testcc_node')
    rospy.spin()

if __name__ == '__main__':
    start()

