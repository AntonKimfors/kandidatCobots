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


#Defining products, a station array and work order array
no_product = "no_product"
product_a = "product_a"    #Assuming 2 different products
product_b = "product_b"
product_in_station = [no_product, no_product, no_product]  #Assuming 3 stations
work_order = []      #First-in-first-out order, gets popped when going to the first station

#Defining messages and organizing them in arrays
cmd_msgs = [Command(), Command(), Command()]
state_msgs = [State(), State(), State()]


def main(args=None):
    #Initializing node, pub and sub
    rclpy.init(args=args)
    node = rclpy.create_node('command_node')
    pub = node.create_publisher(Command, 'cmd1')
    sub = node.create_subscription(State, 'state1', state1_callback)
    #Done with node, pub and sub


    create_work_order()     #Puts products in work_order in a predefined pattern

    while not(product_in_station[0] == no_product and product_in_station[1] == no_product and product_in_station[2] == no_product) or len(work_order) != 0:
        station = 2     #Last station
        while station >= 0:
            print("Kollar station " + str(station))
            time.sleep(1)
            if product_in_station[station] != no_product:
                rclpy.spin_once(node)
                print("Kolla om station " + str(station) + " är klar")
                time.sleep(4)
                if state_msgs[station].finished:
                    product_in_station[station+1] = product_in_station[station]
                    product_in_station[station] = no_product
                    cmd_msgs[station].run = False
                    while not state_msgs[station].init:
                        pub.publish(cmd_msgs[station])
                        rclpy.spin_once(node)
                        print("Pubbar run=false tills jag får init=true")
                        time.sleep(4)

                        #Kan bara sätta en sak som klar om den kan skickas vidare, så kolla stationen framför.
            else:
                if station == 0 and len(work_order) != 0:
                    #Abstrahera till egen metod? Är ju bara för station 0 ändå
                    #Går också att abstrahera hela proceduren med att skicka meddelande, vänta på handskakning och sedan sätta run till true
                    print("Skicka in nästa order till station " + str(station))
                    product_in_station[station] = work_order.pop(0)
                    cmd_msgs[station].command = "Assemble in station " + str(station)
                    cmd_msgs[station].run = False
                    cmd_msgs[station].product_name = product_in_station[station]
                    while state_msgs[station].cmd != cmd_msgs[station].command:
                        pub.publish(cmd_msgs[station])
                        print("Väntar på svar från station " + str(station) + " med meddelandet " + cmd_msgs[station].command)
                        rclpy.spin_once(node)
                        time.sleep(4)
                    cmd_msgs[station].run = True
                    while not state_msgs[station].executing:
                        pub.publish(cmd_msgs[station])
                        rclpy.spin_once(node)
                        print("Pubbar run=true tills jag får executing=true")
                        time.sleep(4)
                else:
                    print("Inget att göra")
                    time.sleep(1)

            station = station - 1
    print(product_in_station)
    print(work_order)

    node.destroy_node()
    rclpy.shutdown()

def state1_callback(state_msg):
    state_msgs[0] = state_msg

def create_work_order():
    work_order.append(product_a)
    #work_order.append(product_b)
    #work_order.append(product_a)


if __name__ == '__main__':
    main()
