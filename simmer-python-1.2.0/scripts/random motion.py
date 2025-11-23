'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial

# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True


############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0




############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False # If true, run this. If false, skip it
while RUN_COMMUNICATION_CLIENT:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    [responses, time_rx] = receive()
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")

# Global list to keep track of all drive commands
drive_history = []
##########Drive Function, this pauses everything until the drive command is complete ##########
def execute_drive(cmd: str):
    """
    Send a drive command to the rover and wait until it completes.

    Args:
        cmd (str): Drive command, e.g., 'w0:1' or 'r0:90'
    """
    # 1️⃣ Record the command in history
    drive_history.append(cmd)
    print("Drive history:", drive_history)
    
    # 2️⃣ Packetize and send the command
    packet_tx = packetize(cmd)
    if not packet_tx:
        print(f"Failed to packetize command: {cmd}")
        return

    transmit(packet_tx)
    completed = False
    while not completed:
        # Poll the rover by re-sending the command to check completion
        packet_tx = packetize("w0:0")
        if packet_tx:
            transmit(packet_tx)

        responses, time_rx = receive()
        if responses and responses[0][1] == 'True':
            completed = True
        else:
            time.sleep(0.05)  # short delay to avoid spamming receive

# Escape 
def recovery_escape():
    """
    Handles the recovery after a 180° turn.
    Moves forward by 12 until an open side is found, then turns.
    """
    escape_found = False
    while not escape_found:
        # Move forward by 12
        execute_drive("w0:12")
        time.sleep(LOOP_PAUSE_TIME)

        # Read side sensors
        sensor_ids = ['u1', 'u2', 'u4', 'u5']
        sensor_data = {}
        for sid in sensor_ids:
            packet_tx = packetize(sid)
            if packet_tx:
                transmit(packet_tx)
                responses, _ = receive()
                try:
                    distance = float(responses[0][1])
                except (ValueError, IndexError):
                    distance = 0
                sensor_data[sid] = distance

        left_avg = (sensor_data['u1'] + sensor_data['u2']) / 2
        right_avg = (sensor_data['u4'] + sensor_data['u5']) / 2

        # Decide turn
        if left_avg > 6 and right_avg <= 6:
            print(f"Escape route found on LEFT: {left_avg:.2f}")
            execute_drive(f"r0:-{TURN_ANGLE}")
            escape_found = True
        elif right_avg > 6 and left_avg <= 6:
            print(f"Escape route found on RIGHT: {right_avg:.2f}")
            execute_drive(f"r0:{TURN_ANGLE}")
            escape_found = True
        elif left_avg > 6 and right_avg > 6:
            # Both sides open → bias to RIGHT
            print(f"Both sides open, biasing turn to RIGHT ({right_avg:.2f} vs {left_avg:.2f})")
            execute_drive(f"r0:{TURN_ANGLE}")
            escape_found = True
        else:
            print("Both sides blocked, moving forward again...")

    return True  # escape complete

############## Main section for the open loop control algorithm ##############
import time

STEP_DISTANCE = 3       # Move 1 unit per step
STEP_ANGLE = 90         # For initial 360° rotation
TURN_ANGLE = 90         # Regular turn
BACK_TURN_ANGLE = 180   # U-turn when trapped
LOOP_PAUSE_TIME = 1     # Pause between actions
STOP_DISTANCE = 4     # Threshold for obstacle detection (inches)
WALL_TOO_CLOSE = 5.0    # Distance indicating wall alongside
OPEN_SIDE_THRESHOLD = 5.0   # distance considered as an open escape path

RUN_DEAD_RECKONING = True
TURNED_AROUND = False # becomes true after a 180deg turn

if RUN_DEAD_RECKONING:
    # ----- PHASE 1: 360° rotation scan -----
    total_rotation = 0
    print("Starting 360° rotation scan...")

    while total_rotation < 360:
        cmd = f"r0:{STEP_ANGLE}"
        packet_tx = packetize(cmd)
        execute_drive(cmd)
        total_rotation += STEP_ANGLE
        time.sleep(LOOP_PAUSE_TIME)

    print("360° rotation complete!")

    # ----- PHASE 2: Forward maze navigation -----
    print("Starting forward navigation...")

    while RUN_DEAD_RECKONING:
        # Read ultrasonic sensors
        sensor_ids = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
        sensor_data = {}

        for sid in sensor_ids:
            packet_tx = packetize(sid)
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                try:
                    distance = float(responses[0][1])
                except (ValueError, IndexError):
                    distance = 999  # assume clear if invalid
                sensor_data[sid] = distance
                print(f"{sid}: {distance:.2f} in")

        # Simplify sensor access
        front_dist = sensor_data['u0']
        left_avg  = (sensor_data['u1'] + sensor_data['u2']) / 2
        right_avg = (sensor_data['u4'] + sensor_data['u5']) / 2
        
        # --- Recovery after 180° turn ---
        if TURNED_AROUND:
            recovery_escape()
            TURNED_AROUND = False
            continue  # go to next iteration after escaping
        # --- Normal Navigation ---
        if front_dist < STOP_DISTANCE:
            print(f"Obstacle ahead ({front_dist:.2f} in)! Deciding turn...")

            # Check if trapped (both sides too close)
            if left_avg < WALL_TOO_CLOSE and right_avg < WALL_TOO_CLOSE:
                print(f"Trapped! Left={left_avg:.2f}, Right={right_avg:.2f} → Turning 180°")
                turn_dir = BACK_TURN_ANGLE
                TURNED_AROUND = True
            else:
                # Turn toward side with more space
                if left_avg > right_avg:
                    turn_dir = -TURN_ANGLE  # left
                    print(f"Turning LEFT ({left_avg:.2f} in vs {right_avg:.2f} in)")
                else:
                    turn_dir = TURN_ANGLE   # right
                    print(f"Turning RIGHT ({right_avg:.2f} in vs {left_avg:.2f} in)")

            cmd = f"r0:{turn_dir}"
            execute_drive(cmd)

        else:
            # Move forward one step
            cmd = f"w0:{STEP_DISTANCE}"

            execute_drive(cmd)

        time.sleep(LOOP_PAUSE_TIME)

