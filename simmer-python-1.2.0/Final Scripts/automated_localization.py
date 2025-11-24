"""
Automated Localization Module

Automatically moves the robot until it achieves unique localization.
Uses w0:3 movements (4 per cell) and execute_cmds_with_safety for alignment.
"""

import time
from execute_cmd import execute_cmds_with_safety


def auto_localize(localizer, transmit_fn, receive_fn, packetize_fn, max_attempts=20):
    """
    Automatically move robot until localized using intelligent movement strategy.
    
    Strategy:
    1. Read sensors to find longest open direction
    2. Move one cell (4x w0:3) in that direction using execute_cmds_with_safety
    3. Check localization with quick
    4. If not localized, rotate to next best direction and repeat
    
    Args:
        localizer: ManualTrackingLocalizer instance
        transmit_fn: Function to transmit commands
        receive_fn: Function to receive responses
        packetize_fn: Function to packetize commands
        max_attempts: Maximum number of cells to move before giving up
    
    Returns:
        tuple: (x, y, orientation) if localized, None if failed
    """
    print("\n" + "="*60)
    print("AUTOMATED LOCALIZATION")
    print("="*60)
    print("Robot will move intelligently until localized...")
    
    # Initial sensor reading and localization attempt
    print("\n--- Initial localization attempt ---")
    result = wait_and_localize(localizer, transmit_fn, receive_fn, packetize_fn)
    
    if result and isinstance(result, tuple):
        print(f"\n✓ Already localized at start!")
        return result
    
    # Main localization loop
    for attempt in range(max_attempts):
        print(f"\n{'='*60}")
        print(f"ATTEMPT {attempt + 1}/{max_attempts}")
        print(f"{'='*60}")
        
        # Read sensors to determine best direction
        print("\n--- Reading sensors to find open direction ---")
        packet = packetize_fn("u0,u1,u2,u3,u4,u5")
        transmit_fn(packet)
        time.sleep(0.15)
        resp, _ = receive_fn()
        
        if not resp or len(resp) < 6:
            print("ERROR: Could not read sensors")
            continue
        
        try:
            u0 = float(resp[0][1])  # front
            u1 = float(resp[1][1])  # left front
            u2 = float(resp[2][1])  # left back
            u3 = float(resp[3][1])  # back
            u4 = float(resp[4][1])  # right back
            u5 = float(resp[5][1])  # right front
            
            # Calculate distances for each direction
            front_dist = u0
            back_dist = u3
            left_dist = max(u1, u2)  # Use best of left sensors
            right_dist = max(u4, u5)  # Use best of right sensors
            
            print(f"Distances: Front={front_dist:.1f}\" Back={back_dist:.1f}\" Left={left_dist:.1f}\" Right={right_dist:.1f}\"")
            
            # Find longest open direction (must be > 12" to move a cell)
            directions = [
                ('front', front_dist, 0),
                ('right', right_dist, 90),
                ('back', back_dist, 180),
                ('left', left_dist, -90)
            ]
            
            # Sort by distance (longest first)
            directions.sort(key=lambda x: x[1], reverse=True)
            
            # Try directions in order of longest distance
            moved = False
            for dir_name, dist, rotation in directions:
                if dist > 12:  # Need at least 12" to move one cell
                    print(f"\n→ Longest open direction: {dir_name} ({dist:.1f}\")")
                    
                    # Rotate to face that direction (if needed)
                    if rotation != 0:
                        print(f"   Rotating {rotation}° to face {dir_name}")
                        cmds = [f"r0:{rotation}"]
                        execute_cmds_with_safety(cmds)
                        
                        # Record turn in localizer
                        localizer.record_turn(rotation)
                        print(f"   ✓ Rotation complete and recorded")
                    
                    # Move one cell using 4x w0:3 commands
                    print(f"   Moving forward 12\" (4x w0:3)")
                    cmds = ['w0:3', 'w0:3', 'w0:3', 'w0:3']
                    execute_cmds_with_safety(cmds)
                    
                    # Record each w0:3 movement
                    for i in range(4):
                        localizer.record_move(3)
                    print(f"   ✓ Movement complete and recorded")
                    
                    moved = True
                    break
            
            if not moved:
                print("\n⚠ All directions blocked (< 12\") - cannot move!")
                print("   Try manually moving robot to open area")
                return None
            
        except (ValueError, IndexError, TypeError) as e:
            print(f"ERROR parsing sensor data: {e}")
            continue
        
        # Check localization after movement
        print("\n--- Checking localization ---")
        result = wait_and_localize(localizer, transmit_fn, receive_fn, packetize_fn)
        
        if result and isinstance(result, tuple):
            print(f"\n{'='*60}")
            print(f"✓ LOCALIZED after {attempt + 1} movements!")
            print(f"{'='*60}")
            return result
        else:
            print(f"   Not yet localized. Candidates remaining: {len(result) if result else 'unknown'}")
            print(f"   Continuing to next movement...")
    
    print(f"\n⚠ Failed to localize after {max_attempts} attempts")
    return None


def wait_and_localize(localizer, transmit_fn, receive_fn, packetize_fn, timeout=5):
    """
    Wait for robot to finish moving, then run localization.
    
    Args:
        localizer: ManualTrackingLocalizer instance
        transmit_fn: Function to transmit commands
        receive_fn: Function to receive responses  
        packetize_fn: Function to packetize commands
        timeout: Maximum seconds to wait for movement to complete
    
    Returns:
        Localization result (tuple if unique, list if multiple candidates, None if error)
    """
    # Wait for movement to complete by checking w0 status
    start_time = time.time()
    while time.time() - start_time < timeout:
        packet_check = packetize_fn('w0')
        transmit_fn(packet_check)
        time.sleep(0.1)
        check_resp, _ = receive_fn()
        
        if check_resp and check_resp[0] and check_resp[0][1] == 'True':
            # Robot is ready
            break
        time.sleep(0.1)
    else:
        print("⚠ Warning: Timeout waiting for robot to be ready")
    
    # Small delay to ensure sensors are stable
    time.sleep(0.2)
    
    # Run localization
    return localizer.quick_localize()


# Example usage
if __name__ == "__main__":
    from comms import transmit, receive, packetize
    from localization_submodule import create_manual_localizer
    from astar import MAZE_TO_DROPOFF
    
    ROWS = 4
    COLS = 8
    
    # Create localizer
    localizer = create_manual_localizer(
        MAZE_TO_DROPOFF, ROWS, COLS,
        transmit, receive, packetize,
        wall_thresh=6.0,
        min_dist=0.5
    )
    
    # Run automated localization
    result = auto_localize(localizer, transmit, receive, packetize)
    
    if result:
        x, y, orientation = result
        print(f"\nFinal position: ({x}, {y}) facing {orientation}°")
    else:
        print("\nAutomated localization failed")