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
        self.last_run_recieved = data.run
        
        #if a new command is recieved, update current command and refresh the view
        if (self.last_command_recieved != data.command):
            self.last_command_recieved = data.command
            self.current_cmd = data.command
            agv_state.cmd = self.current_cmd
            self.refresh_view()
        if (self.last_run_recieved == False and self.current_state == "finished"):
            self.current_state = "init"
            agv_state.state = self.current_state
            
        #always anser a publish with a ublish.
        self.ccpub.publish(agv_state)


    def callback_button_state(self, data):

        # if Run is set to False and Current status is finished, set status to init
        if (self.last_run_recieved == False and self.current_state == "finished"):
            self.current_state = "init"
            self.refresh_view()
        # if current status is init and x is pressed, set status to executing and pub
        if (self.current_state == "init"):
            if (data.xpress == True):
                self.current_state = "executing"
                self.current_cmd = self.last_command_recieved
                self.last_sent_cmd = self.current_state
                agv_state.state = self.current_state
                agv_state.cmd = self.current_cmd
                self.ccpub.publish(agv_state)
                self.refresh_view()
            #Add accept button ? 


        # if current status is Exec and B is pressed, set status to finished and pub
        if (self.current_state == "executing"):
            if (data.bpress == True):
                self.current_state = "finished"
                self.last_sent_cmd = self.current_state
                agv_state.state = self.current_state
                self.current_cmd = ""
                agv_state.cmd = self.current_cmd
                self.ccpub.publish(agv_state) 
                self.refresh_view()

    def refresh_view(self):
         print('\rCurrent command: ' + self.current_cmd )
         print('\rCurrent State: ' + self.current_state)
         print('\rRun: ' + str(self.last_run_recieved))
      

if __name__ == '__main__':
    try:
        agv_comms()
    except rospy.ROSInterruptException:
        pass

