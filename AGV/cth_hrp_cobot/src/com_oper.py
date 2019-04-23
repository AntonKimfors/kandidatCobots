#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from cth_hrp_cobot.msg import ButtonPressed
from cth_hrp_cobot.msg import Command
from cth_hrp_cobot.msg import State
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus
from os import system



class agv_comms():
    def __init__ (self):
        # Initial values
        self.battAVolt = 0.0
        self.battBVolt = 0.0

        # Base States:
        self.agv_state = State()
        self.last_command_recieved = ""
        self.last_run_recieved = False
        self.last_product_recieved = ""
        self.current_cmd = ""
        self.current_state = "init"
        self.last_command_recieved = ""
        self.last_product_recieved = ""
        self.last_run_recieved = False
        self.last_sent_cmd = ""
        self.agv_state.message = ""
        self.agv_state.cmd = ""
        self.agv_state.state = ""

        #sensor states:
        self.mowerInternalState = 0


        self.ccpub = rospy.Publisher('/AGV_state', State)
        #subscribing to the comandcenter topic as well as the buttionstates from the teleop.
        rospy.Subscriber("/commandcenter", Command, self.callback_commandcenter)
        rospy.Subscriber("/button_state", ButtonPressed, self.callback_button_state)
        rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
        rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
        # starts the node
        rospy.init_node('com_oper_node')
        rospy.spin()
    def mowerStateToString(self, x):
        return {
            0:'OFF',
            1:'WAIT_SAFETY_PIN',
            2:'STOPPED',
            3:'FATAL_ERROR',
            4:'PENDING_START',
            5:'PAUSED',
            6:'IN_OPERATION',
            7:'RESTRICTED',
            8:'ERROR'
        }.get(x,'UNKNONW_VALUE')



    def callback_sensor_status(self,data):
        if (self.mowerInternalState != data.mowerInternalState):
            self.mowerInternalState = data.mowerInternalState
            self.refresh_view()
        
    def callback_battery_status(self,data):
        self.battAVolt = data.batteryAVoltage/1000.0
        self.battBVolt = data.batteryBVoltage/1000.0
        self.refresh_view()


    def callback_commandcenter(self, data):
        # update current and lastrecieved product to the recieved data - might skip
        self.last_run_recieved = data.run
        
        #if a new command is recieved, update current command and refresh the view
        if (self.last_command_recieved != data.command):
            self.last_command_recieved = data.command
            self.current_cmd = data.command
            self.agv_state.cmd = self.current_cmd
            self.refresh_view()
            self.ccpub.publish(self.agv_state)
        # if Run is set to False and Current status is finished, set status to init
        if (self.last_run_recieved == False and self.current_state == "finished"):
            self.current_state = "init"
            self.agv_state.state = self.current_state
            self.refresh_view()
            self.ccpub.publish(self.agv_state)  
        #always answer a publish with a publish with current state. might need to change
        # self.ccpub.publish(agv_state)

    def callback_button_state(self, data):
        # if current status is init and x is pressed, set status to executing and pub
        # should a check that a command has been given, meaning either button to accept or auto accept mission
        if (self.current_state == "init" and self.current_cmd != ""):  # more requirements needed 
            if (data.xpress == True):
                self.current_state = "executing"
                self.current_cmd = self.last_command_recieved
                self.last_sent_cmd = self.current_state
                self.agv_state.state = self.current_state
                self.agv_state.cmd = self.current_cmd
                self.ccpub.publish(self.agv_state)
                self.refresh_view()
            #Add accept button ? 


        # if current status is Exec and B is pressed, set status to finished and pub
        if (self.current_state == "executing"):
            if (data.bpress == True):
                self.current_state = "finished"
                self.last_sent_cmd = self.current_state
                self.agv_state.state = self.current_state
                self.current_cmd = ""
                self.agv_state.cmd = self.current_cmd
                self.ccpub.publish(self.agv_state) 
                self.refresh_view()

    def refresh_view(self):
        system('clear')
        print('Last sent command:' + self.last_sent_cmd)
        print('Current command: ' + self.current_cmd )
        print('Current State: ' + self.current_state)
        msg = 'batA %.1f V  batB %.1f V' % (self.battAVolt,self.battBVolt)
        print(msg)
        mowerState = self.mowerStateToString(self.mowerInternalState)
        print('Mower State: ' + mowerState)
      

if __name__ == '__main__':
    try:
        agv_comms()
    except rospy.ROSInterruptException:
        pass
