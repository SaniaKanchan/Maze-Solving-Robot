"""
Manual Rover Control Client with Localization Interface

Commands:
  w0:<inches>   - Move forward (e.g., w0:12)
  r0:<degrees>  - Rotate (e.g., r0:90 for right, r0:-90 for left)
  localize      - Run full localization sequence
  quick         - Quick single-reading localization
  sensors       - Read all sensors once
  maze          - Display the maze
  pos           - Show current estimated position
  thresh        - Show wall detection thresholds
  thresh <val>  - Set wall threshold (e.g., thresh 7)
  help          - Show commands
  quit          - Exit
"""

import sys
import socket
import time
from datetime import datetime
import serial
from chris_local import create_manual_localizer

# ============== CONFIGURATION ==============
### Network Setup ###
HOST = '127.0.0.1'
PORT_TX = 61200
PORT_RX = 61201

### Serial Setup ###
BAUDRATE = 9600
PORT_SERIAL = 'COM9'
TIMEOUT_SERIAL = 1.5

### Packet Framing ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Mode Selection ###
SIMULATE = False  # True = SimMeR (TCP), False = Arduino (Serial)

### Maze Definition ###
MAZE = [
    [0, 0, 0, 0, 1, 0, 1, 0],  # row 0
    [0, 0, 1, 0, 0, 0, 0, 0],  # row 1
    [0, 1, 0, 1, 1, 0, 1, 0],  # row 2
    [0, 0, 0, 0, 0, 0, 1, 0]   # row 3
]
ROWS = len(MAZE)
COLS = len(MAZE[0])

### Wall Detection Thresholds ###
WALL_THRESH = 6.0  # inches - wall detected if distance < this
MIN_DIST = 0.5     # inches - ignore readings below this

### Current Position (updated by localization) ###
current_pos = {'x': None, 'y': None, 'orientation': None}

# ============== INITIALIZATION ==============
if SIMULATE:
    SOURCE = 'SimMeR'
    TRANSMIT_PAUSE = 0.1
else:
    SOURCE = f'serial device {PORT_SERIAL}'
    TRANSMIT_PAUSE = 0
    try:
        SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
        print(f"Serial connection established on {PORT_SERIAL}")
    except serial.SerialException:
        print(f'Serial connection was refused.')
        print(f'Ensure {PORT_SERIAL} is the correct port and nothing else is connected.')
        sys.exit()

# ============== COMMUNICATION FUNCTIONS ==============
def clear_serial(delay_time: float = 0):
    """Wait some time and then clear the serial buffer."""
    if not SIMULATE and SER.in_waiting:
        time.sleep(delay_time)
        dumped = SER.read(SER.in_waiting)
        print(f'Clearing Serial... Dumped: {dumped}')

def transmit(data):
    """Selects whether to use serial or tcp for transmitting."""
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def transmit_tcp(data):
    """Send a command over the TCP connection."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset. Is simmer.py running?')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def transmit_serial(data):
    """Transmit a command over a serial connection."""
    clear_serial()
    SER.write(data.encode('ascii'))

def receive():
    """Selects whether to use serial or tcp for receiving."""
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

def receive_tcp():
    """Receive a reply over the TCP connection."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')
            return [[False], datetime.now().strftime("%H:%M:%S")]
    return [[False], datetime.now().strftime("%H:%M:%S")]

def receive_serial():
    """Receive a reply over a serial connection."""
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
    
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def depacketize(data_raw: str):
    """Take a raw string and verify it's a complete packet, return data as list."""
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)
    
    if start >= 0 and end >= start:
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]
        
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
    """Take a message and packetize it with start and end framing."""
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)
    if not check_fail:
        return FRAMESTART + data + FRAMEEND
    return False

def response_string(cmds: str, responses_list: list):
    """Build a string showing responses to transmitted commands."""
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    out_string = ''
    for cmd_id, resp in zip(cmd_list, responses_list):
        if resp and len(resp) >= 2:
            sgn = '=' if cmd_id == resp[0] else '!='
            chk = '✓' if cmd_id == resp[0] else 'X'
            out_string += f'  cmd {cmd_id} {sgn} {resp[0]} {chk}, response "{resp[1]}"\n'
    return out_string

# ============== DISPLAY FUNCTIONS ==============
def show_maze():
    print("\nMaze Layout (·=free, █=obstacle):")
    print("    " + " ".join(str(c) for c in range(COLS)))
    print("   " + "-" * (COLS * 2 + 1))
    for r in range(ROWS):
        row_str = " ".join('█' if MAZE[r][c] else '·' for c in range(COLS))
        print(f" {r} |{row_str}|")
    print("   " + "-" * (COLS * 2 + 1))
    print("\nOrientation: 0°=East(→) 90°=South(↓) 180°=West(←) 270°=North(↑)")

def show_position():
    if current_pos['x'] is not None:
        ori_name = localizer.directions.get(current_pos['orientation'], 'unknown')
        print(f"\nEstimated Position: ({current_pos['x']}, {current_pos['y']})")
        print(f"Orientation: {current_pos['orientation']}° ({ori_name})")
    else:
        print("\nPosition unknown - run 'localize' first")

def show_threshold():
    print(f"\nWall Detection Thresholds:")
    print(f"  WALL_THRESH = {localizer.wall_thresh} inches")
    print(f"  MIN_DIST = {localizer.min_dist} inches")

def show_help():
    print("""
Commands:
  w0:<inches>   - Move forward (e.g., w0:12) - automatically tracked
  r0:<degrees>  - Rotate (e.g., r0:90) - automatically tracked
  quick         - Localize using sensor reading + movement history
  sensors       - Read sensors only (no localization)
  status        - Show tracking status (movements, candidates)
  reset         - Clear movement history and start fresh
  maze          - Show maze layout
  pos           - Show current position estimate
  thresh        - Show wall detection thresholds
  thresh <val>  - Set wall threshold (e.g., thresh 7)
  help          - This message
  quit/exit     - Exit program
  
Any other input will be sent as a raw command (not tracked).
""")

# ============== CREATE LOCALIZER ==============
localizer = create_manual_localizer(
    MAZE, ROWS, COLS,
    transmit, receive, packetize,
    WALL_THRESH, MIN_DIST
)

# ============== MAIN LOOP ==============
def main():
    global current_pos
    
    print(f"--- Starting Remote Control Client for {SOURCE} ---")
    print("="*50)
    show_help()
    show_maze()
    show_threshold()
    
    while True:
        try:
            cmd = input("\n> ").strip()
            cmd_lower = cmd.lower()
            
            if not cmd:
                continue
            
            # Exit commands
            elif cmd_lower in ['quit', 'exit', 'q']:
                print("Exiting...")
                break
            
            # Help and display commands
            elif cmd_lower == 'help':
                show_help()
            elif cmd_lower == 'maze':
                show_maze()
            elif cmd_lower == 'pos':
                show_position()
            elif cmd_lower == 'thresh':
                show_threshold()
            elif cmd_lower.startswith('thresh '):
                try:
                    val = float(cmd_lower.split()[1])
                    localizer.set_threshold(wall_thresh=val)
                except (ValueError, IndexError):
                    print("Usage: thresh <number>")
            
            # Localization commands
            elif cmd_lower == 'sensors':
                localizer.read_sensors()
            elif cmd_lower == 'quick':
                result = localizer.quick_localize()
                if result and isinstance(result, tuple):
                    x, y, o = result
                    current_pos = {'x': x, 'y': y, 'orientation': o}
            elif cmd_lower == 'status':
                localizer.get_status()
            elif cmd_lower == 'reset':
                localizer.reset_history()
                current_pos = {'x': None, 'y': None, 'orientation': None}
            
            # Movement commands (w0, r0) - TRACKED
            elif cmd_lower.startswith('w0:'):
                packet_tx = packetize(cmd)
                if packet_tx:
                    print(f"Sending: {cmd}")
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    if responses[0]:
                        print(f"At time '{time_rx}' received from {SOURCE}:")
                        print(response_string(cmd, responses))
                        # Extract inches and record movement
                        try:
                            inches = float(cmd.split(':')[1])
                            localizer.record_move(inches)
                        except (ValueError, IndexError):
                            print("Warning: Could not parse distance, movement not tracked")
                    else:
                        print(f"At time '{time_rx}': Malformed packet or no response")
                else:
                    print("Invalid command (contains forbidden characters)")
            
            elif cmd_lower.startswith('r0:'):
                packet_tx = packetize(cmd)
                if packet_tx:
                    print(f"Sending: {cmd}")
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    if responses[0]:
                        print(f"At time '{time_rx}' received from {SOURCE}:")
                        print(response_string(cmd, responses))
                        # Extract degrees and record turn
                        try:
                            degrees = float(cmd.split(':')[1])
                            localizer.record_turn(degrees)
                        except (ValueError, IndexError):
                            print("Warning: Could not parse degrees, turn not tracked")
                    else:
                        print(f"At time '{time_rx}': Malformed packet or no response")
                else:
                    print("Invalid command (contains forbidden characters)")
            
            # Raw command (anything else)
            else:
                packet_tx = packetize(cmd)
                if packet_tx:
                    print(f"Sending raw: {cmd}")
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    if responses[0]:
                        print(f"At time '{time_rx}' received from {SOURCE}:")
                        print(response_string(cmd, responses))
                    else:
                        print(f"At time '{time_rx}': Malformed packet or no response")
                else:
                    print("Invalid command (contains forbidden characters)")
                    
        except KeyboardInterrupt:
            print("\nInterrupted")
            break
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
    # Cleanup
    if not SIMULATE:
        SER.close()
        print("Serial connection closed")
    print("Client shutting down.")

if __name__ == "__main__":
    main()