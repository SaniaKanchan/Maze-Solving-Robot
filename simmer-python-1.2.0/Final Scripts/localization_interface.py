"""
Localization Interface Module

This module provides a complete interactive localization interface
that can be called from other scripts.

Main function: run_manual_localization(maze, rows, cols, transmit, receive, packetize)
Returns: (x, y, orientation) or (None, None, None) if cancelled
"""

import time
from localization_submodule import create_manual_localizer

def run_manual_localization(maze, rows, cols, transmit_fn, receive_fn, packetize_fn, 
                            wall_thresh=6.0, min_dist=0.5):
    """
    Run the full interactive localization interface.
    
    Args:
        maze: 2D maze array (0=free, 1=obstacle)
        rows: Number of rows
        cols: Number of columns
        transmit_fn: Function to transmit data
        receive_fn: Function to receive data
        packetize_fn: Function to packetize commands
        wall_thresh: Wall detection threshold (inches)
        min_dist: Minimum distance threshold (inches)
    
    Returns:
        tuple: (x, y, orientation) if localized and 'run' typed
               (None, None, None) if quit/cancelled
    """
    print("\n" + "="*60)
    print("MANUAL LOCALIZATION PHASE")
    print("Type 'help' for commands, 'run' when ready to execute path")
    print("="*60)
    
    # Create localizer
    localizer = create_manual_localizer(
        maze, rows, cols,
        transmit_fn, receive_fn, packetize_fn,
        wall_thresh, min_dist
    )
    
    current_pos = {'x': None, 'y': None, 'orientation': None}
    movement_in_progress = False  # Track if robot is moving
    
    def show_help():
        print("""
Commands:
  w0:<inches>   - Move forward (e.g., w0:12) - automatically tracked
  r0:<degrees>  - Rotate (e.g., r0:90 right, r0:-90 left) - automatically tracked
  quick         - Localize using sensor reading + movement history
  sensors       - Read sensors only (no localization)
  status        - Show tracking status (movements, candidates)
  reset         - Clear movement history and start fresh
  maze          - Show maze layout
  pos           - Show current position estimate
  thresh        - Show wall detection thresholds
  thresh <val>  - Set wall threshold (e.g., thresh 7)
  help          - This message
  run           - Finish localization and start path execution
  quit/exit     - Exit program
""")
    
    def show_maze():
        print("\nMaze Layout (·=free, █=obstacle):")
        print("    " + " ".join(str(c) for c in range(cols)))
        print("   " + "-" * (cols * 2 + 1))
        for r in range(rows):
            row_str = " ".join('█' if maze[r][c] else '·' for c in range(cols))
            print(f" {r} |{row_str}|")
        print("   " + "-" * (cols * 2 + 1))
        print("\nOrientation: 0°=East(→) 90°=South(↓) 180°=West(←) 270°=North(↑)")
    
    show_help()
    show_maze()
    
    while True:
        try:
            cmd = input("\n> ").strip()
            cmd_lower = cmd.lower()
            
            if not cmd:
                continue
            
            # Exit commands
            elif cmd_lower in ['quit', 'exit', 'q']:
                print("Exiting localization...")
                return None, None, None
            
            # Help and display
            elif cmd_lower == 'help':
                show_help()
            elif cmd_lower == 'maze':
                show_maze()
            elif cmd_lower == 'status':
                localizer.get_status()
                if current_pos['x'] is not None:
                    print(f"Current Position: ({current_pos['x']}, {current_pos['y']}) facing {current_pos['orientation']}°")
            elif cmd_lower == 'pos':
                if current_pos['x'] is not None:
                    ori_name = localizer.directions[current_pos['orientation']]
                    print(f"\nCurrent Position: ({current_pos['x']}, {current_pos['y']})")
                    print(f"Orientation: {current_pos['orientation']}° ({ori_name})")
                else:
                    print("\nPosition unknown - run 'quick' to localize")
            elif cmd_lower == 'thresh':
                print(f"\nWall Detection Thresholds:")
                print(f"  WALL_THRESH = {localizer.wall_thresh} inches")
                print(f"  MIN_DIST = {localizer.min_dist} inches")
                print(f"\nFor 12-inch cells with robot centered:")
                print(f"  Distance to adjacent wall ≈ 6 inches")
            elif cmd_lower.startswith('thresh '):
                try:
                    val = float(cmd_lower.split()[1])
                    localizer.set_threshold(wall_thresh=val)
                except (ValueError, IndexError):
                    print("Usage: thresh <number>")
            elif cmd_lower == 'reset':
                localizer.reset_history()
                current_pos = {'x': None, 'y': None, 'orientation': None}
            
            # Localization
            elif cmd_lower == 'sensors':
                localizer.read_sensors()
            elif cmd_lower == 'quick':
                # Check if robot is still moving
                if movement_in_progress:
                    print("⚠ Movement in progress - checking if complete...")
                    packet_check = packetize_fn('w0')
                    transmit_fn(packet_check)
                    time.sleep(0.1)
                    check_resp, _ = receive_fn()
                    
                    if check_resp[0][1] == 'True':
                        print("✓ Movement complete!")
                        movement_in_progress = False
                    else:
                        print("⚠ Robot still moving - wait before localizing")
                        continue
                
                result = localizer.quick_localize()
                if result and isinstance(result, tuple):
                    x, y, o = result
                    current_pos = {'x': x, 'y': y, 'orientation': o}
                    print(f"\n{'='*50}")
                    print("✓ Localized! Type 'run' to start path execution.")
                    print(f"{'='*50}")
            
            # Movement commands
            elif cmd_lower.startswith('w0:'):
                packet = packetize_fn(cmd)
                if packet:
                    print(f"Sending: {cmd}")
                    transmit_fn(packet)
                    time.sleep(0.5)
                    resp, _ = receive_fn()
                    
                    if resp and resp[0]:
                        print(f"Response: {resp}")
                        # Only record if command succeeded
                        if len(resp[0]) >= 2 and resp[0][1] == 'True':
                            movement_in_progress = True
                            print("→ Movement started - wait for completion before 'quick'")
                            
                            try:
                                inches = float(cmd.split(':')[1])
                                localizer.record_move(inches)
                                print("✓ Movement recorded")
                            except (ValueError, IndexError):
                                print("Warning: Could not parse distance")
                        else:
                            print("⚠ Command failed - movement NOT recorded")
                    else:
                        print("⚠ No valid response - movement NOT recorded")
                else:
                    print("Invalid command format")
            
            elif cmd_lower.startswith('r0:'):
                packet = packetize_fn(cmd)
                if packet:
                    print(f"Sending: {cmd}")
                    transmit_fn(packet)
                    time.sleep(0.5)
                    resp, _ = receive_fn()
                    
                    if resp and resp[0]:
                        print(f"Response: {resp}")
                        # Only record if command succeeded
                        if len(resp[0]) >= 2 and resp[0][1] == 'True':
                            movement_in_progress = True
                            print("→ Rotation started - wait for completion before 'quick'")
                            
                            try:
                                degrees = float(cmd.split(':')[1])
                                localizer.record_turn(degrees)
                                print("✓ Turn recorded")
                            except (ValueError, IndexError):
                                print("Warning: Could not parse degrees")
                        else:
                            print("⚠ Command failed - turn NOT recorded")
                    else:
                        print("⚠ No valid response - turn NOT recorded")
                else:
                    print("Invalid command format")
            
            # Finish localization and return to caller
            elif cmd_lower == 'run':
                if current_pos['x'] is None:
                    print("ERROR: Must localize first! Use 'quick' command.")
                    continue
                print("\n" + "="*60)
                print("Localization complete! Returning to main script...")
                print(f"Position: ({current_pos['x']}, {current_pos['y']}) facing {current_pos['orientation']}°")
                print("="*60)
                return current_pos['x'], current_pos['y'], current_pos['orientation']
            
            # Unknown command
            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' for available commands")
                
        except KeyboardInterrupt:
            print("\nInterrupted")
            return None, None, None
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()


# ============== STANDALONE MODE ==============
# If run directly, use with default TCP settings
if __name__ == "__main__":
    import sys
    import socket
    from datetime import datetime
    import serial
    
    # Configuration
    HOST = '127.0.0.1'
    PORT_TX = 61200
    PORT_RX = 61201
    BAUDRATE = 9600
    PORT_SERIAL = 'COM11'
    TIMEOUT_SERIAL = 1.5
    FRAMESTART = '['
    FRAMEEND = ']'
    SIMULATE = True
    TRANSMIT_PAUSE = 0.1 if SIMULATE else 0
    
    # Maze
    MAZE = [
        [0, 0, 0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 1, 0, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0]
    ]
    ROWS = len(MAZE)
    COLS = len(MAZE[0])
    
    # Serial setup
    SER = None
    if SIMULATE:
        SOURCE = 'SimMeR'
    else:
        SOURCE = f'serial device {PORT_SERIAL}'
        try:
            SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
            print(f"Serial connection established on {PORT_SERIAL}")
        except serial.SerialException:
            print(f'Serial connection failed on {PORT_SERIAL}')
            sys.exit()
    
    # Communication functions
    def clear_serial(delay_time=0):
        if SER and SER.in_waiting:
            time.sleep(delay_time)
            dumped = SER.read(SER.in_waiting)
            print(f'Clearing Serial... Dumped: {dumped}')
    
    def transmit(data):
        if SIMULATE:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.connect((HOST, PORT_TX))
                    s.send(data.encode('ascii'))
                except (ConnectionRefusedError, ConnectionResetError, TimeoutError) as e:
                    print(f'Tx Error: {e}')
        else:
            clear_serial()
            SER.write(data.encode('ascii'))
        time.sleep(TRANSMIT_PAUSE)
    
    def receive():
        if SIMULATE:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                try:
                    s.connect((HOST, PORT_RX))
                    raw = s.recv(1024).decode('ascii')
                    if raw:
                        return [depacketize(raw), datetime.now().strftime("%H:%M:%S")]
                except (ConnectionRefusedError, ConnectionResetError, TimeoutError) as e:
                    print(f'Rx Error: {e}')
            return [[False], datetime.now().strftime("%H:%M:%S")]
        else:
            start = time.time()
            raw = ''
            while time.time() < start + TIMEOUT_SERIAL:
                if SER.in_waiting:
                    c = SER.read().decode('ascii')
                    raw += c
                    if c == FRAMEEND:
                        break
            print(f'Raw response: {raw}')
            if raw:
                return [depacketize(raw), datetime.now().strftime("%H:%M:%S")]
            return [[False], datetime.now().strftime("%H:%M:%S")]
    
    def depacketize(raw):
        start = raw.find(FRAMESTART)
        end = raw.find(FRAMEEND)
        if start >= 0 and end > start:
            data = raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
            cmd_list = []
            for item in data:
                parts = item.split(':', 1)
                match len(parts):
                    case 0:
                        cmd_list.append(['', ''])
                    case 1:
                        cmd_list.append([parts[0], ''])
                    case _:
                        cmd_list.append(parts[:2])
            return cmd_list
        return [[False, '']]
    
    def packetize(data):
        forbidden = [FRAMESTART, FRAMEEND, '\n']
        if any(c in data for c in forbidden):
            return False
        return FRAMESTART + data + FRAMEEND
    
    # Run standalone
    print("="*60)
    print("Manual Rover Control (Standalone Mode)")
    print(f"Mode: {SOURCE}")
    print("="*60)
    
    result = run_manual_localization(
        MAZE, ROWS, COLS,
        transmit, receive, packetize
    )
    
    if result[0] is not None:
        x, y, orientation = result
        print(f"\nFinal localized position: ({x}, {y}) facing {orientation}°")
    else:
        print("\nLocalization cancelled or failed")
    
    if SER:
        SER.close()