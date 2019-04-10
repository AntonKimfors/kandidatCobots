#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from agv_trans_comm.msg import ButtonPressed
from agv_trans_comm.msg import Command
from agv_trans_comm.msg import State



class agv_comms():
    def __init__ (self):
        global agv_state 
        agv_state = State()
        self.last_command_recieved = ""
        self.last_run_recieved = False
        self.last_product_recieved = ""
        self.current_cmd = ""
        self.current_state = "init"
        self.last_command_recieved = ""
        self.last_product_recieved = ""
        self.last_run_recieved = False
        agv_state.message = ""
        agv_state.cmd = ""
        agv_state.state = ""
        #self.main()
        self.ccpub = rospy.Publisher('/AGV_state', State)
        #subscribing to the comandcenter topic as well as the buttionstates from the teleop.
        rospy.Subscriber("/commandcenter", Command, self.callback_commandcenter)
        rospy.Subscriber("/button_state", ButtonPressed, self.callback_button_state)
        # starts the node
        rospy.init_node('AGVClassCom')
        rospy.spin()



    def callback_commandcenter(self, data):
        # update current and lastrecieved product to the recieved data - might skip
        #last_product_recieved = data.product_name
        #agv_state.product_name = last_product_recieved
        self.last_run_recieved = data.run
        
        #if a new command is recieved, update current command and refresh the view
        print ("last_command_recieved: " + self.last_command_recieved + '\n')
        print ("data.command: " + data.command + '\n')
        print (last_command_recieved != data.command)
        if (self.last_command_recieved != data.command):
            self.last_command_recieved = data.command
            self.current_cmd = data.command
            agv_state.cmd = current_cmd
            self.refresh_view()
        
        if (self.last_run_recieved == False and self.current_state == "finished"):
            print('in reset')
            self.current_state = "init"
            agv_state.state = self.current_state
        #always anser a publish with a publish.
        self.ccpub.publish(agv_state)


    def callback_button_state(self, data):
        #agv_state = State()
        # if X is pressed, set status and send
        #print ("xpressed: " + data.xpress + '\n')
        #print ("bpressed: " + data.bpress + '\n')
        if (self.current_state == "init"):
            if (data.xpress == True):
                print('sending execute')
                self.current_state = "executing"
                self.current_cmd = self.last_command_recieved
                agv_state.state = self.current_state
                agv_state.cmd = self.current_cmd
        
        # if B is pressed, set status and send
        if (self.current_state == "executing"):
            if (data.bpress == True):
                print('sending finished')
                self.current_state = "finished"
                agv_state.state = self.current_state
                self.current_cmd = ""
                agv_state.cmd = self.current_cmd
        #Publish with new changes
        self.ccpub.publish(agv_state)  # kanske flytta in

    def refresh_view(self):
        print('Last command: ' + self.last_command_recieved)
      

if __name__ == '__main__':
    try:
        agv_comms()
        rospy.init_node('AGVClassCom')
    except rospy.ROSInterruptException:
        pass
