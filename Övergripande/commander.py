#!/usr/bin/env python

# ----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
# ----------------------------------------------------------------------------------------------------------------------#
#   Christian Karlsson
#   Kandidatarbete COBOTS
#   V.1.0.0.
# ----------------------------------------------------------------------------------------------------------------------#

import rclpy
import time

from commander_msgs.msg import Command
from commander_msgs.msg import State

# Used for parsing additional arguments to callback function
from functools import partial

# Generating product order for testing purposes
import generateProductOrder as gpo


# Defining pubs and subs
pubs = []
subs = []

# Defining products, a station array and work order array
products = ["product_a", "product_b"]
NUMBEROFPRODUCTS = 10

# First-in-first-out order, gets popped when going to the first station
work_order = []

# Defining message arrays
cmd_msgs = []
state_msgs = []

# Defining the different states allowed
init = 'init'
executing = 'executing'
finished = 'finished'

# Defining the node
node = None

# Number of stations, hardcoded/predefined
noOfStations = 3

# Sleep time where sleep is implemented
sleep_time = 1


def main(args=None):
    input("Press Enter to continue...")

    global node

    # Initializes node, creates pubs, subs and initializes the message arrays
    node = initialize(noOfStations)

    # Generate product order
    product_order = gpo.generate_product_order(
        NUMBEROFPRODUCTS, products)

    # Puts products in work_order in a hardcoded/predefined pattern
    create_work_order(product_order)

    all_stations_empty = is_all_stations_empty()

    # While there is something in at least one of the stations or if the work
    # order is not empty
    while not all_stations_empty or len(work_order) != 0:

        # From last station index down to 0
        for station in range(noOfStations-1, -1, -1):

            print("Checking station " + str(station))
            rclpy.spin_once(node)
            time.sleep(sleep_time)

            # If station is not empty
            if cmd_msgs[station].product_name != '':

                print("Checking if station " + str(station) + " is done")
                time.sleep(sleep_time)

                if check_state(station) == finished:
                    # Last station => no station to send forward to
                    if station == noOfStations - 1:
                        station_done(station)

                    # Not last station => check forward
                    elif check_state(station+1) == init:
                        cmd_msgs[station+1].product_name = (cmd_msgs[station].
                                                            product_name)

                        # Should a station be done before the next station has
                        # started on the product?
                        station_done(station)
                        send_command(station+1)

                    else:
                        print("Station " + str(station) + " is done but the \
                             station ahead is not ready")

                else:
                    print("Station " + str(station) + " is not done")

            else:
                # If nothing in the first station but the work_order is not
                # empty
                if station == 0 and len(work_order) != 0:
                    if check_state(station) != init:

                        print("Station " + str(station) + " is not ready for \
                             another work order")

                    else:
                        print("Send in the next order to station " +
                              str(station))
                        cmd_msgs[station].product_name = work_order.pop(0)
                        send_command(station)
                else:
                    print("Nothing to do")
                    time.sleep(sleep_time)

        all_stations_empty = is_all_stations_empty()

    print(work_order)

    node.destroy_node()
    rclpy.shutdown()


def state_callback(station, state_msg: str):
    state_msgs[station] = state_msg


# Initializes node, creates pubs and subs and initializes the message arrays
def initialize(noOfStations: int):
    rclpy.init()
    node = rclpy.create_node('command_node')

    for i in range(noOfStations):
        pubs.append(node.create_publisher(Command, 'cmd' + str(i)))

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subs.append(node.create_subscription(State, 'state' + str(i),
                    partial(state_callback, i)))
    return node


# Puts products in work_order in a hardcoded/predefined pattern
def create_work_order(Product_order: list):
    for product in Product_order:
        work_order.append(product)


# Returns True if all stations are empty, False otherwise
def is_all_stations_empty():
    rclpy.spin_once(node)
    for cmd_msg in cmd_msgs:
        if cmd_msg.product_name != '':
            return False

    return True


# Checks and returns the state of the station. Throws exception if state is
# not recognized
def check_state(station):
    state = state_msgs[station].state
    if state == init or state == executing or state == finished:
        return state
    else:
        raise Exception("Unknown state on station " + str(station) + ": " +
                        state)


# Sets a station to init with handshake
def station_done(station):
    cmd_msgs[station].product_name = ''
    cmd_msgs[station].run = False

    while check_state(station) != init:
        pubs[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=false until state=init on station " + str(station))
        time.sleep(sleep_time)


# Sends command to station and puts run to true when handshake is established
def send_command(station):
    cmd_msgs[station].command = "Assemble in station " + str(station)
    cmd_msgs[station].run = False

    # Waiting for handshake for the command
    while state_msgs[station].cmd != cmd_msgs[station].command:
        pubs[station].publish(cmd_msgs[station])
        print("Waiting on station " + str(station) + " with the message " +
              cmd_msgs[station].command)
        rclpy.spin_once(node)
        time.sleep(sleep_time)

    cmd_msgs[station].run = True

    while check_state(station) != executing:
        pubs[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        print("Pubing run=true until state=executing on station " +
              str(station))
        time.sleep(sleep_time)


if __name__ == '__main__':
    main()
