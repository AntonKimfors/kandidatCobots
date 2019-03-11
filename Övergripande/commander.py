#!/usr/bin/env python

#----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
#----------------------------------------------------------------------------------------------------------------------#
    # Christian Karlsson
    # Kandidatarbete COBOTS
    # V.1.0.0.
#----------------------------------------------------------------------------------------------------------------------#

import rclpy
import time

from commander_msgs.msg import Command
from commander_msgs.msg import State

from functools import partial   #Used for parsing arguments to callback function

#Defining pubs and subs
pubs = []
subs = []

#Defining products, a station array and work order array
product_a = "product_a"    #Assuming 2 different products
product_b = "product_b"
product_in_station = []                                                                             #Is this necessary?
work_order = []      #First-in-first-out order, gets popped when going to the first station

#Defining message arrays
cmd_msgs = []
state_msgs = []

node = None

sleep_time = 1

def main(args=None):
    global node

    noOfStations = 3
    node = init(noOfStations)

    create_work_order()     #Puts products in work_order in a predefined pattern

    while not(product_in_station[0] == None and product_in_station[1] == None and product_in_station[2] == None) or len(work_order) != 0:
        #Skulle kunna ha en while(True) med en loop inuti som loopar igenom alla states och sätter en variabel därefter. Efter den innersta loopen en if-sats som breaker den yttersta.

        for station in range(noOfStations-1, -1, -1):

            print("Checking station " + str(station))
            time.sleep(sleep_time)
            rclpy.spin_once(node)

            if product_in_station[station] != None:

                print("Checking if station " + str(station) + " is done")
                time.sleep(sleep_time)

                if state_msgs[station].finished:
                    if station == noOfStations - 1:     #Last station => no station to send forward to
                        station_done(station)

                    elif state_msgs[station+1].init:    #Not last station => check forward
                        product_in_station[station+1] = product_in_station[station]
                        station_done(station)           #Should a station be done before the next station has started on the product?
                        send_command(station+1)

                    else:
                        print("Station " + str(station) + " is done but the station ahead is not ready")

                else:
                    print("Station " + str(station) + " is not done")

            else:
                if station == 0 and len(work_order) != 0:
                    if not state_msgs[station].init:

                        print("Station " + str(station) + " is not ready for another work order")

                    else:
                        print("Send in the next order to station " + str(station))
                        product_in_station[station] = work_order.pop(0)
                        send_command(station)
                else:
                    print("Nothing to do")
                    time.sleep(sleep_time)

    print(product_in_station)
    print(work_order)

    node.destroy_node()
    rclpy.shutdown()

def init(j):
    rclpy.init()
    node = rclpy.create_node('command_node')

    for i in range(j):
        pubs.append(node.create_publisher(Command, 'cmd' + str(i)))

        product_in_station.append(None)

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subs.append(node.create_subscription(State, 'state' + str(i), partial(state_callback, i)))
    return node

def state_callback(station, state_msg):
    state_msgs[station] = state_msg

def create_work_order():
    work_order.append(product_a)
    #work_order.append(product_b)
    #work_order.append(product_a)

def station_done(station):
    product_in_station[station] = None
    cmd_msgs[station].run = False
    while not state_msgs[station].init:
        pubs[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=false until init=true on station " + str(station))
        time.sleep(sleep_time)

def send_command(station):
    cmd_msgs[station].command = "Assemble in station " + str(station)
    cmd_msgs[station].run = False
    cmd_msgs[station].product_name = product_in_station[station]
    while state_msgs[station].cmd != cmd_msgs[station].command:
        pubs[station].publish(cmd_msgs[station])
        print("Waiting on station " + str(station) + " with the message " + cmd_msgs[station].command)
        rclpy.spin_once(node)
        time.sleep(sleep_time)
    cmd_msgs[station].run = True
    while not state_msgs[station].executing:
        pubs[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=true until executing=true on station " + str(station))
        time.sleep(sleep_time)


if __name__ == '__main__':
    main()
