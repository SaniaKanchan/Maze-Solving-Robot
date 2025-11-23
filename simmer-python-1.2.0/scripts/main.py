import math
from queue import PriorityQueue
import socket
import time
from datetime import datetime
import serial
from Astar import astar, path_to_commands, visualize_maze
from Localization import perform_initial_localization

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

### Maze and Movement Constants ###
MAZE = [
    [0, 0, 0, 0, 1, 0, 1, 0],  # row 0
    [0, 0, 1, 0, 0, 0, 0, 0],  # row 1
    [0, 1, 0, 1, 1, 0, 1, 0],  # row 2
    [0, 0, 0, 0, 0, 0, 1, 0]   # row 3
]
ROWS = len(MAZE)
COLS = len(MAZE[0])
CELL_SIZE_INCHES = 12
MOVE_INCREMENT = 12


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
    
############### MAIN SCRIPT CONTROL ##############
def main():
    print("="*60)
    print("SimMeR Navigation System")
    print(f"Source: {SOURCE}")
    print("="*60)

    # Step 1: Initial Localization
    print("\nPerforming initial localization...")
    x, y, orientation, localizer = perform_initial_localization(
        MAZE, 
        ROWS, 
        COLS,
        transmit,
        receive,
        packetize,
    )
    
    if x is None or y is None or orientation is None:
        print("Failed to localize! Exiting...")
        return
    
    print(f"\nLocalized position: ({x}, {y}) facing {orientation}°")

    # Wait for any pending localization commands to complete
    print("Waiting for localization commands to complete...")
    time.sleep(2)  # Allow time for last movement to finish
    # Try to confirm completion
    responses, _ = receive()
    if responses and len(responses) > 0:
        if isinstance(responses[0], list) and len(responses[0]) > 1 and responses[0][1] == 'True':
            print("Confirmed: Last command completed")
        else:
            print("Warning: Could not confirm last command status")
    else:
        print("Warning: No response received")

    # Step 2: Get start and end positions from user
    print("\nStarting A* path planning...")
    start_x = input(f"Enter start x position (default {x}): ")
    start_y = input(f"Enter start y position (default {y}): ")
    end_x = input("Enter end x position: ")
    end_y = input("Enter end y position: ")
    
    start_pos = (int(start_x) if start_x else x, int(start_y) if start_y else y)
    goal_cell = (int(end_x), int(end_y))
    
    print(f"\nPlanning path from {start_pos} to {goal_cell}")
    
    path = astar(start_pos, goal_cell, MAZE, ROWS, COLS)
    if not path:
        print("No path found to goal! Exiting...")
        return

    # Visualize planned path
    visualize_maze(path, MAZE, ROWS, COLS)
    
    # Step 3: Execute Path with Relocalization
    commands = path_to_commands(path, start_angle=orientation)
    print(f"\nExecuting {len(commands)} commands with periodic relocalization...")
    
    RELOCALIZE_INTERVAL = 4  # Relocalize every N commands
    for i, cmd in enumerate(commands):
        print(f"\nCommand {i+1}/{len(commands)}: {cmd}")
        
        # Execute command
        packet_tx = packetize(cmd)
        transmit(packet_tx)
        
        # Wait for completion
        while True:
            responses, _ = receive()
            if responses and responses[0][1] == 'True':
                break
            time.sleep(0.1)
        
        # Periodic relocalization
        if (i + 1) % RELOCALIZE_INTERVAL == 0:
            print("Performing relocalization check...")
            time.sleep(1)  # Ensure rover has stopped before sensor reading
            new_x, new_y, new_orientation = localizer.relocalize()
            
            if new_x is not None:
                # Check if we're significantly off course
                expected_x, expected_y = path[i // RELOCALIZE_INTERVAL]
                if abs(new_x - expected_x) > 0.5 or abs(new_y - expected_y) > 0.5:
                    print("Significant position error detected! Replanning...")
                    
                    # Replan from current position
                    new_path = astar((new_x, new_y), goal_cell, MAZE, ROWS, COLS)
                    if new_path:
                        path = new_path
                        commands = path_to_commands(path, start_angle=new_orientation)
                        print("Path replanned successfully")
                        # Start executing new commands
                        continue
                    else:
                        print("Failed to replan path! Stopping...")
                        return
            else:
                print("Warning: Relocalization failed, continuing with current plan")

    print("\nNavigation complete!")
    
    # Final position check
    final_x, final_y, final_orientation = localizer.relocalize()
    if final_x is not None:
        print(f"Final position: ({final_x}, {final_y}) facing {final_orientation}°")
    else:
        print("Warning: Final position check failed")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        if not SIMULATE:
            SER.close()
