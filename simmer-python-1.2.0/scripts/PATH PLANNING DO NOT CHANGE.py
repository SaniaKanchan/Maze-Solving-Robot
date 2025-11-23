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

import math
from queue import PriorityQueue
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

import math
from queue import PriorityQueue

# --- Maze definition (4x8 grid, 1=blocked, 0=free) ---
# row 0 = top, col 0 = left
MAZE = [
    [0, 0, 0, 0, 1, 0, 1, 0],  # row 0
    [0, 0, 1, 0, 0, 0, 0, 0],  # row 1
    [0, 1, 0, 1, 1, 0, 1, 0],  # row 2
    [0, 0, 0, 0, 0, 0, 1, 0]   # row 3
]
ROWS = len(MAZE)
COLS = len(MAZE[0])

# Cell size and movement parameters (all in inches)
CELL_SIZE_INCHES = 12  # Each maze cell is 12 inches (center to center)
MOVE_INCREMENT = 3     # Move in 3-inch increments

# --- A* Node ---
class Node:
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g  # cost so far
        self.h = h  # heuristic
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

# --- Heuristic: Manhattan distance ---
def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

# --- Generate neighbors ---
def neighbors(node):
    dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # left, up, right, down
    result = []
    for dx, dy in dirs:
        nx, ny = node.x + dx, node.y + dy
        # MAZE[row][col] = MAZE[y][x]
        if 0 <= nx < COLS and 0 <= ny < ROWS and MAZE[ny][nx] == 0:
            result.append((nx, ny))
    return result

# --- A* search ---
def astar(start, goal):
    """
    Find shortest path from start to goal using A*.
    
    Args:
        start: (x, y) tuple where x=column, y=row
        goal: (x, y) tuple where x=column, y=row
    
    Returns:
        List of (x, y) coordinates from start to goal, or None if no path
    """
    open_set = PriorityQueue()
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])
    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.g + start_node.h
    open_set.put((start_node.f, start_node))
    closed = set()

    while not open_set.empty():
        _, current = open_set.get()
        
        if (current.x, current.y) in closed:
            continue
            
        if (current.x, current.y) == (goal_node.x, goal_node.y):
            # Reconstruct path
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]
            
        closed.add((current.x, current.y))
        
        for nx, ny in neighbors(current):
            if (nx, ny) in closed:
                continue
            g_cost = current.g + 1
            h_cost = heuristic(Node(nx, ny), goal_node)
            neighbor_node = Node(nx, ny, g_cost, h_cost, current)
            open_set.put((neighbor_node.f, neighbor_node))
            
    return None

# --- Convert path to incremental moves ---
def path_to_commands(path, start_angle=180, cell_size=CELL_SIZE_INCHES, increment=MOVE_INCREMENT):
    """
    Convert A* cell path to incremental movement commands.
    Since coordinates refer to cell centers, moving from one cell to another is exactly 12 inches.
    
    Args:
        path: List of (x, y) tuples from A*
        start_angle: Initial robot orientation (0=right, 90=down, 180=left, 270=up)
        cell_size: Distance between cell centers in inches (default: 12)
        increment: Distance per move command in inches (default: 3)
    
    Returns:
        List of command strings (e.g., ["r0:90", "w0:3", ...])
    """
    commands = []
    angle = start_angle
    prev_x, prev_y = path[0]
    steps_per_cell = cell_size // increment

    for x, y in path[1:]:
        dx, dy = x - prev_x, y - prev_y
        
        # Determine desired direction
        if dx == 1:
            desired_angle = 0       # right
        elif dx == -1:
            desired_angle = 180     # left
        elif dy == 1:
            desired_angle = 90      # down
        elif dy == -1:
            desired_angle = 270     # up
        else:
            continue  # no move

        # Compute rotation needed (shortest path)
        turn = (desired_angle - angle) % 360
        if turn > 180:
            turn -= 360
            
        if turn != 0:
            commands.append(f"r0:{turn}")
            angle = desired_angle

        # Move forward 12 inches (center to center) in increments
        for _ in range(steps_per_cell):
            commands.append(f"w0:{increment}")

        prev_x, prev_y = x, y

    return commands

# --- Execute commands with relocalization ---
def execute_with_relocalization(commands):
    """
    Execute movement commands with relocalization after each step.
    Waits for each command to complete before sending the next one.
    A command returns 'True' when completed, 'False' if rejected (another command is running).
    
    Args:
        commands: List of command strings to execute
    
    Returns:
        bool: True if all commands executed successfully, False otherwise
    """
    print(f"\nExecuting {len(commands)} commands with relocalization...")
    
    for i, cmd in enumerate(commands, 1):
        print(f"[{i}/{len(commands)}] Command: {cmd}")
        
        # Keep trying to send the command until it's accepted and completed
        command_completed = False
        
        while not command_completed:
            # Packetize and send the command
            packet_tx = packetize(cmd)
            if not packet_tx:
                print(f"  ERROR: Failed to packetize command: {cmd}")
                return False
            
            transmit(packet_tx)
            time.sleep(0.1)  # Brief delay for transmission
            
            # Receive response
            responses, time_rx = receive()
            
            # Check response
            if responses and len(responses) > 0 and len(responses[0]) > 1:
                status = responses[0][1]
                
                if status == 'True':
                    # Command completed successfully
                    command_completed = True
                    print(f"  ✓ Completed at {time_rx}")
                    time.sleep(0.1)  # Brief pause before next command
                    
                elif status == 'False':
                    # Command rejected (another command is running), retry
                    print(f"  ⟳ Waiting for previous command to finish...")
                    time.sleep(0.2)  # Wait before retrying
                else:
                    print(f"  WARNING: Unexpected response: {status}")
                    time.sleep(0.1)
            else:
                print(f"  WARNING: Invalid response format")
                time.sleep(0.1)
        
        # TODO: Relocalization logic here
        # After each move completes, you would:
        # 1. Get current position from sensors/localization
        # 2. Compare with expected position
        # 3. If error is significant, replan from current position
        #
        # Example:
        # current_pos = get_localized_position()
        # expected_pos = calculate_expected_position()
        # if distance(current_pos, expected_pos) > THRESHOLD:
        #     print("  Drift detected! Replanning...")
        #     new_path = astar(current_pos, goal)
        #     new_commands = path_to_commands(new_path, current_angle)
        #     return execute_with_relocalization(new_commands)
    
    return True

# --- Visualize maze with path ---
def visualize_maze(path=None):
    """Print the maze with optional path overlay."""
    print("\nMaze visualization:")
    print(f"  {ROWS}x{COLS} grid (0=free, 1=obstacle)")
    
    path_set = set(path) if path else set()
    
    for y in range(ROWS):
        row_str = f"  Row {y}: "
        for x in range(COLS):
            if (x, y) in path_set:
                if (x, y) == path[0]:
                    row_str += "S "  # Start
                elif (x, y) == path[-1]:
                    row_str += "G "  # Goal
                else:
                    row_str += "• "  # Path
            elif MAZE[y][x] == 1:
                row_str += "█ "  # Obstacle
            else:
                row_str += "· "  # Free
        print(row_str)

# --- Main execution ---
if __name__ == "__main__":
    print("="*60)
    print("A* Path Planning for SimMeR with Relocalization")
    print(f"Source: {SOURCE}")
    print("="*60)
    
    # Define start and goal (x, y) where x=column, y=row
    start_cell = (7, 3)  # col 7, row 3
    goal_cell = (0, 2)   # col 0, row 2
    start_orientation = 270  # facing up initially
    
    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal:  (col={goal_cell[0]}, row={goal_cell[1]})")
    print(f"Note: Cell centers are 12 inches apart")
    
    # Find path using A*
    print("\nSearching for path with A*...")
    path = astar(start_cell, goal_cell)
    
    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")
        
        # Visualize
        visualize_maze(path)
        
        # Convert to commands
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"\nGenerated {len(cmds)} movement commands:")
        
        # Group and display commands
        rotation_cmds = [c for c in cmds if c.startswith('r')]
        move_cmds = [c for c in cmds if c.startswith('w')]
        print(f"  - {len(rotation_cmds)} rotation commands")
        print(f"  - {len(move_cmds)} movement commands")
        print("\nCommand sequence:")
        for i, cmd in enumerate(cmds, 1):
            print(f"  {i:2d}. {cmd}")
        
        # Execute if desired
        print("\n" + "="*60)
        user_input = input("Execute commands with relocalization? (y/n): ")
        if user_input.lower() == 'y':
            success = execute_with_relocalization(cmds)
            if success:
                print("\n✓ Navigation complete!")
            else:
                print("\n✗ Navigation failed!")
        else:
            print("\nExecution cancelled by user.")
            
    else:
        print("✗ No path found!")
        print("Possible reasons:")
        print("  - Start or goal is in an obstacle")
        print("  - No valid path exists between start and goal")
        visualize_maze()