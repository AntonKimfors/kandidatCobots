#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from cth_hrp_cobot_legacy.msg import ButtonPressed
from cth_hrp_cobot_legacy.msg import Command
from cth_hrp_cobot_legacy.msg import State


# Author: Marcus Lindvarn
'''
def __init__ (self)
    last_command_recieved = ""
    last_product_recieved = ""
    last_run_recieved = False
    current_cmd = ""
    current_state = "init"
    agv_state.message = ""
    agv_state.cmd = ""
    agv_state.state = ""
'''
def callback_commandcenter(data):
    # update current and lastrecieved product to the recieved data - might skip
    #last_product_recieved = data.product_name
    #agv_state.product_name = last_product_recieved
    last_run_recieved = data.run
    
    #if a new command is recieved, update current command and refresh the view
    print ("last_command_recieved: " + last_command_recieved + '\n')
    print ("data.command: " + data.command + '\n')
    print (last_command_recieved != data.command)
    if (last_command_recieved != data.command):
        last_command_recieved = data.command
        current_cmd = data.command
        agv_state.cmd = current_cmd
        refresh_view(self)
    
    if (last_run_recieved == False and current_state == "finished"):
        print('in reset')
        global current_state
        current_state = "init"
        agv_state.state = current_state
    #always anser a publish with a publish.
    ccpub.publish(agv_state)


def callback_button_state(data):
    #agv_state = State()
    # if X is pressed, set status and send
    #print ("xpressed: " + data.xpress + '\n')
    #print ("bpressed: " + data.bpress + '\n')
    if (current_state == "init"):
        if (data.xpress == True):
            print('sending execute')
            current_state = "executing"
            current_cmd = last_command_recieved
            agv_state.state = current_state
            agv_state.cmd = current_cmd
    
    # if B is pressed, set status and send
    if (current_state == "executing"):
        if (data.bpress == True):
            print('sending finished')
            current_state = "finished"
            agv_state.state = current_state
            current_cmd = ""
            agv_state.cmd = current_cmd
    #Publish with new changes
    ccpub.publish(agv_state)

def refresh_view(self):
    print('Last command: ' + last_command_recieved)

# Intializes everything
def start():    
    global agv_state 
    agv_state = State()
    #__init__(self)
    global last_command_recieved
    global last_run_recieved
    global last_product_recieved
    global current_cmd
    global ccpub
    last_command_recieved = ""
    last_product_recieved = ""
    last_run_recieved = False
    current_cmd = ""
    global current_state 
    current_state= "init"
    agv_state.message = ""
    agv_state.cmd = ""
    agv_state.state = ""
    ccpub = rospy.Publisher('/AGV_state', State)
    #subscribing to the comandcenter topic as well as the buttionstates from the teleop.
    rospy.Subscriber("/commandcenter", Command, callback_commandcenter)
    rospy.Subscriber("/button_state", ButtonPressed, callback_button_state)
    # starts the node
    rospy.init_node('AGVComms_node')
    rospy.spin()

if __name__ == '__main__':
    start()

