#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys, termios, tty, select, os, threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from cth_hrp_cobot.msg import ButtonPressed
from cth_hrp_cobot.msg import Command
from cth_hrp_cobot.msg import State
from std_msgs.msg import UInt16
from am_driver.msg import SensorStatus
from am_driver.msg import BatteryStatus
from os import system


def mowerStateToString(x):
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


def controlStateToString(x):
  return {
    0:'UNDEFINED',
    1:'IDLE',
    2:'INIT',
    3:'MANUAL',
    4:'RANDOM',
    5:'PARK',
  }.get(x,'UNKNONW_VALUE')



#define	AM_STATE_UNDEFINED     0x0
#define	AM_STATE_IDLE          0x1
#define	AM_STATE_INIT          0x2
#define	AM_STATE_MANUAL        0x3
#define	AM_STATE_RANDOM        0x4
#define	AM_STATE_PARK          0x5

my_mutex = threading.Lock() 
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
        self.last_sent_cmd = ""
        agv_state.message = ""
        agv_state.cmd = ""
        agv_state.state = ""
        #self.main()
        self.ccpub = rospy.Publisher('/AGV_state', State)
        #subscribing to the comandcenter topic as well as the buttionstates from the teleop.
        rospy.Subscriber("/commandcenter", Command, self.callback_commandcenter)
        rospy.Subscriber("/button_state", ButtonPressed, self.callback_button_state)
        self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
            rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
    rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
        # starts the node
        rospy.init_node('AGVClassCom')
        rospy.spin()

# Save terminal settings
    self.settings = termios.tcgetattr(sys.stdin)
    # Initial values
    self.inc_ratio = 0.1
    self.speed = np.array([0.3, 1.0])
    self.command = np.array([0, 0])
    self.update_rate = 10   # Hz
    self.alive = True
    self.battAVolt = 0.0
    self.battBVolt = 0.0
    # Setup publishers
    self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.pub_mode = rospy.Publisher('/cmd_mode', UInt16, queue_size=1)
    rospy.Subscriber('/sensor_status',SensorStatus,self.callback_sensor_status)
    rospy.Subscriber('/battery_status',BatteryStatus,self.callback_battery_status)
    
    self.searching = False
    
    self.shapeNum = 0x20
    self.operationalMode = 0;
    self.sensorStatus = 0;
    self.mowerInternalState = 0;
    self.controlState = 0;

    self.last_terminalWidth = 0
   
    
    
 
  def fini(self):
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
 

  def callback_sensor_status(self,data):
    if (self.operationalMode != data.operationalMode) or (self.sensorStatus != data.sensorStatus) or (self.mowerInternalState != data.mowerInternalState) or (self.controlState != data.controlState):
      self.operationalMode = data.operationalMode
      self.sensorStatus = data.sensorStatus
      self.mowerInternalState = data.mowerInternalState
      self.controlState = data.controlState
      self.showstatuslines()
     
  def callback_battery_status(self,data):
    self.battAVolt = data.batteryAVoltage/1000.0
    self.battBVolt = data.batteryBVoltage/1000.0
    self.showstatuslines()
	


    
  def showstatuslines(self):
    my_mutex.acquire()

    mode =''
    if self.operationalMode==0:
      mode = 'Offline'
    elif self.operationalMode == 1:
      mode = 'Manual '
    elif self.operationalMode == 2:
      mode = 'Random '

		 
	#define HVA_SS_HMB_CTRL 0x0001
    #define HVA_SS_OUTSIDE 0x0002
    #define HVA_SS_COLLISION 0x0004
    #define HVA_SS_LIFTED 0x0008
    #define HVA_SS_TOO_STEEP 0x0010
    #define HVA_SS_PARKED 0x0020
    #define HVA_SS_IN_CS 0x0040
    #define HVA_SS_USER_STOP 0x0080
    #define HVA_SS_CFG_NEEDED 0x0100
    #define HVA_SS_DISC_ON 0x0200
    #define HVA_SS_LOOP_ON 0x0400
	
    status = ''
    if self.sensorStatus & 0x0001:
      status = status + 'HMB_CTRL '
    if self.sensorStatus & 0x0002:
      status = status + 'Outside '
    if self.sensorStatus & 0x0004:
      status = status + 'Collision '
    if self.sensorStatus & 0x0008:
      status = status + 'Lifted '
    if self.sensorStatus & 0x0010:
      status = status + 'TooSteep '
    if self.sensorStatus & 0x0020:
      status = status + 'PARKED '
    if self.sensorStatus & 0x0040:
      status = status + 'IN_CS '
    if self.sensorStatus & 0x0800:
      status = status + 'Charging '
    if self.sensorStatus & 0x0080:
      status = status + 'USER_STOP '
    if self.sensorStatus & 0x0100:
      status = status + 'CFG NEEDED'
    if self.sensorStatus & 0x0200:
      status = status + 'DISC_ON '
    else:
      status = status + 'DISC_OFF '
    if self.sensorStatus & 0x0400:
      status = status + 'LOOP_ON '
    else:
      status = status + 'LOOP_OFF '
      

    
    rows, term_width = os.popen('stty size', 'r').read().split()

    if term_width < self.last_terminalWidth:
    # Terminal shrink destroys status message, show usage again
      self.loginfo('\n\n\n\n\n\n\n\n\n\n\n')
      self.print_usage()
    self.last_terminalWidth = term_width

    self.move_cursor_one_line_up() 
    self.move_cursor_one_line_up()
    self.move_cursor_one_line_up()

    msg = 'Linear %.2f  angular %.2f     batA %.1f V  batB %.1f V' % (self.speed[0],self.speed[1],self.battAVolt,self.battBVolt)
    self.logstatusline(msg)

    if self.mowerInternalState != 0:
      # We are controlling via am_driver_safe
      mowerState = mowerStateToString(self.mowerInternalState)
      controlState = controlStateToString(self.controlState)
      msg = 'ControlMode: %s            MowerState: %s' % (controlState,mowerState)
    else:
      # We are controlling via am_driver_legacy
      msg = 'ControlMode: %s            ' % (mode)
		
    
    self.logstatusline(msg)

    msg = 'Status: %s' % (status)
    self.logstatusline(msg)

    my_mutex.release()

  def run(self):
    try:
      self.init()
      self.print_usage()
      self.showstatuslines()

      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        ch = self.get_key()
        self.process_key(ch)
        self.update()
        self.showstatuslines()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass
    finally:
      self.fini()

    def callback_commandcenter(self, data):
        # update current and lastrecieved product to the recieved data - might skip
        self.last_run_recieved = data.run
        
        #if a new command is recieved, update current command and refresh the view
        if (self.last_command_recieved != data.command):
            self.last_command_recieved = data.command
            self.current_cmd = data.command
            agv_state.cmd = self.current_cmd
            self.refresh_view()
            self.ccpub.publish(agv_state)
        # if Run is set to False and Current status is finished, set status to init
        if (self.last_run_recieved == False and self.current_state == "finished"):
            self.current_state = "init"
            agv_state.state = self.current_state
            self.refresh_view()
            self.ccpub.publish(agv_state)  
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
        system('clear')
        print('Last sent command:' + self.last_sent_cmd)
        print('Current command: ' + self.current_cmd )
        print('Current State: ' + self.current_state)
        print('Run: ' + str(self.last_run_recieved))
      

if __name__ == '__main__':
    try:
        agv_comms()
    except rospy.ROSInterruptException:
        pass
