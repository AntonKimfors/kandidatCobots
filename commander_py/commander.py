#!/usr/bin/env python

# ----------------------------------------------------------------------------------------------------------------------#
# authors, description, version
# ----------------------------------------------------------------------------------------------------------------------#
#   Christian Karlsson
#   Kandidatarbete COBOTS
#   V.3.0.0.
# ----------------------------------------------------------------------------------------------------------------------#

import rclpy
import time
import os
import configparser

from commander_msgs.msg import Command
from commander_msgs.msg import State

# import commander_py.generatelog as log # for ubuntu
import generatelog as log               # for everything else

# Used for parsing additional arguments to callback function
from functools import partial

# Read from config file:
CONFIGFILEPATH = "{}/{}".format(os.path.dirname(__file__), 'config.ini')
Config = configparser.ConfigParser()
Config.read(CONFIGFILEPATH)

# Defining publishers and subscribers
publishers = []
subscribers = []

# Defining products, a station array and work order array
products = Config.get('Product', 'Producttypes')
NUMBEROFPRODUCTS = len(products)

# Input file for productorder:
product_order_input = '/product_order_input.txt'

# First-in-first-out order, gets popped when going to the first station
work_order = []

# Defining message arrays
cmd_msgs = []
state_msgs = []

# Defining the different states allowed
INIT = 'init'
EXECUTING = 'executing'
FINISHED = 'finished'

# Defining the node
node = None

# Number of stations, hardcoded/predefined
NO_OF_STATIONS = Config.get('Station', 'numberOfStations')

# Sleep time where sleep is implemented
SLEEP_TIME = Config.get('General', 'SLEEP_TIME')

# Messages for the 3 different stations and transport
# COMMAND_ARRAY = ["Kit", "Assemble", "Screw", "Move transport to station"]
COMMAND_ARRAY = Config.get('Station', 'Stationtypes').split()

# Tracks which station the transport is on
transport_pos = 0


def main(args=None):
    input("Press Enter to continue...")

    # Initializes node, creates publishers, subscribers and initializes the
    # message arrays
    global node
    node = initialize(NO_OF_STATIONS)

    # Read product order
    dirname = os.path.dirname(__file__)
    path = "{}{}".format(dirname, product_order_input)
    file = open(path, "r")
    product_order = file.readlines()
    file.close

    # Puts products in work_order in a hardcoded/predefined pattern
    create_work_order(product_order)

    # While there is something in at least one of the stations or if the work
    # order is not empty
    all_stations_empty = is_all_stations_empty()

    while not all_stations_empty or len(work_order) != 0:

        # From last station index down to 0
        for station in range(NO_OF_STATIONS-1, -1, -1):
            msg = "Checking station {}".format(str(station))
            log.write_to_log(msg)
            print(msg)
            rclpy.spin_once(node)
            time.sleep(SLEEP_TIME)

            # If station is not empty
            if cmd_msgs[station].product_name:
                msg = "Checking if station {} is done".format(str(station))
                log.write_to_log(msg)
                print(msg)
                time.sleep(SLEEP_TIME)

                if (check_state(station) == FINISHED and
                        transport_pos == station):
                    # Last station => no station to send forward to
                    if station == NO_OF_STATIONS - 1:
                        station_done(station)

                    # Not last station => check forward
                    elif check_state(station+1) == INIT:
                        cmd_msgs[station+1].product_name = (cmd_msgs[station].
                                                            product_name)

                        # Should a station be done before the next station has
                        # started on the product?
                        station_done(station)
                        send_command(station+1)

                    else:
                        msg = "Station {} is done but the \
                              station ahead is not ready".format(str(station))
                        log.write_to_log(msg)
                        print(msg)

                else:
                    msg = '''Station {} is not done, or the transport is not in
                            position'''.format(str(station))
                    log.write_to_log(msg)
                    print(msg)

            else:
                # If nothing in the first station but the work_order is not
                # empty
                if station == 0 and len(work_order) != 0:
                    if check_state(station) != INIT:
                        msg = "Station {} not ready for new work order".format(
                            str(station))
                        log.write_to_log(msg)
                        print(msg)

                    else:
                        msg = "Send in the next order to station {}".format(
                              str(station))
                        log.write_to_log(msg)
                        print(msg)
                        cmd_msgs[station].product_name = work_order.pop(
                            0).rstrip()
                        send_command(station)
                else:
                    msg = "Nothing to do"
                    log.write_to_log(msg)
                    print(msg)
                    time.sleep(SLEEP_TIME)

        all_stations_empty = is_all_stations_empty()
    
    print(work_order)

    node.destroy_node()
    rclpy.shutdown()


def state_callback(station, state_msg: str):
    '''
    Callback message from the designated station \n
    Input:
        station - Which stations state message called
        state_msg (string) - Which message to call
    '''
    state_msgs[station] = state_msg


def initialize(NO_OF_STATIONS: int):
    '''
    Initializes node, creates publishers and subscribers and initializes the
    message arrays. \n
    Input:
        NO_OF_STATIONS (int) - Number of stations in the production system.
    Returns:
        node - Command_node
    '''
    rclpy.init()
    node = rclpy.create_node('command_node')

    for i in range(NO_OF_STATIONS):
        publishers.append(node.create_publisher(
            Command, 'cmd{}'.format(str(i))))

        cmd_msgs.append(Command())
        state_msgs.append(State())

        subscribers.append(node.create_subscription(State, 'state{}'.format(
            str(i)), partial(state_callback, i)))

    # Below is only for the transport
    publishers.append(node.create_publisher(Command, 'cmdA'))
    cmd_msgs.append(Command())
    state_msgs.append(State())
    subscribers.append(node.create_subscription(State, 'stateA', partial(
        state_callback, NO_OF_STATIONS)))

    return node


# Puts products in work_order in a hardcoded/predefined pattern
def create_work_order(Product_order: list):
    '''
    Appends the work_order with the given product_order. \n
    Input:
        Product_order (list) - Product order to be produced
    '''
    for product in Product_order:
        work_order.append(product)


def is_all_stations_empty():
    '''
    Check if all stations are empty. \n
    Returns True or false
    '''
    rclpy.spin_once(node)
    for cmd_msg in cmd_msgs:
        if cmd_msg.product_name != '':
            return False

    return True


def check_state(station: int):
    '''
    Checks and returns the state of the station. Prints message if state is
    not recognized \n
    Input:
        station (int) - Which station to check the state for.
    Returns:
        state - State message of the given station
    '''
    state = state_msgs[station].state.lower()
    if not state_status(state):
        msg = "Unknown state on station {}: {}".format(str(station), state)
        log.write_to_log(msg)
        print(msg)

    return state


def state_status(state):
    ''' Check station states against the preapproved states:
        (init, executing, finished) \n
    State: Station state \n
    Returns: (bool) True or false depending on if the input state is found
             in approvedStatus
    '''
    acceptedStatus = [INIT, EXECUTING, FINISHED, '']  # '' to enable empty msgs
    if state in acceptedStatus:
        return True
    else:
        return False


def station_done(station: int):
    '''
    Sets a station to init with handshake \n
    Input:
        station (int) - which station is done
    '''
    cmd_msgs[station].command = ''
    cmd_msgs[station].run = False
    cmd_msgs[station].product_name = ''

    msg = "Pubing run=false until state=init on station {}".format(
        str(station))
    log.write_to_log(msg)
    print(msg)

    while check_state(station) != INIT:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)

    move_transport(station)


def move_transport(station: int):
    ''' Moves the transport to the next station \n
    Input:
        station (int) - Current station
    '''
    if station >= NO_OF_STATIONS - 1:
        next_station = 0
    else:
        next_station = station + 1

    msg = "Waiting for transport to be in init state"
    log.write_to_log(msg)
    print(msg)

    while check_state(NO_OF_STATIONS) != INIT:
        time.sleep(SLEEP_TIME/2)
        rclpy.spin_once(node)
    msg = "Moving transport to station {}".format(next_station)
    log.write_to_log(msg)
    print(msg)
    cmd_msgs[NO_OF_STATIONS].command = "{}: {}".format(COMMAND_ARRAY[
        NO_OF_STATIONS], str(next_station))
    send_command(NO_OF_STATIONS)

    msg = "Waiting for transport to finish"
    log.write_to_log(msg)
    print(msg)

    while check_state(NO_OF_STATIONS) == EXECUTING:
        time.sleep(SLEEP_TIME/2)
        rclpy.spin_once(node)

    cmd_msgs[NO_OF_STATIONS].command = ''
    cmd_msgs[NO_OF_STATIONS].run = False
    cmd_msgs[NO_OF_STATIONS].product_name = ''

    msg = "Pubing run=false until state=init on transport"
    log.write_to_log(msg)
    print(msg)

    while check_state(NO_OF_STATIONS) != INIT:
        publishers[NO_OF_STATIONS].publish(cmd_msgs[NO_OF_STATIONS])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)

    global transport_pos
    transport_pos = next_station


# Sends command to station and puts run to true when handshake is established
def send_command(station: int):
    '''Sends command to station and puts run to true when handshake is
    established \n
    Input:
        station (int) - Current station
    '''
    # cmd_msgs[station].command = "Assemble in station {}".format(str(station))
    if station != NO_OF_STATIONS:
        cmd_msgs[station].command = COMMAND_ARRAY[station]

    cmd_msgs[station].run = False

    # Waiting for handshake for the command
    msg = "Waiting on station {} with the message {}".format(
        str(station), cmd_msgs[station].command)
    log.write_to_log(msg)
    print(msg)

    while state_msgs[station].cmd != cmd_msgs[station].command:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)

    cmd_msgs[station].run = True
    msg = "Pubing run=true until state=executing on station {}".format(
              str(station))
    log.write_to_log(msg)
    print(msg)
    while check_state(station) != EXECUTING:
        publishers[station].publish(cmd_msgs[station])
        rclpy.spin_once(node)
        time.sleep(SLEEP_TIME/2)


if __name__ == '__main__':
    main()
