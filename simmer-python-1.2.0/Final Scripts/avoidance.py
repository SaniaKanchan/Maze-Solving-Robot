from comms import transmit, receive, packetize, SOURCE
import time

# Sensors: u1, u2 = LEFT side (u1 front, u2 rear)
#          u4, u5 = RIGHT side (u4 rear, u5 front)
# Positive r0 = clockwise (CW)

def bootwallalign():
    """
    Robust initial alignment - rotates until both sensors on one side detect a wall,
    then aligns to that wall. Run once when rover is placed in maze.
    """
    align_threshold = 0.1
    distance_threshold = 5.0
    max_search_rotations = 36  # Max 360° search (36 x 10° = 360°)
    rotation_count = 0
    aligned = False
    wall_found = False
    
    print("=== BOOT WALL ALIGNMENT STARTED ===")
    
    # === PHASE 1: SEARCH FOR WALL (BOTH SENSORS ON ONE SIDE) ===
    while not wall_found:
        # Get ultrasonic readings
        cmd = 'u1,u2,u4,u5'
        packet = packetize(cmd)
        transmit(packet)
        [responses, time_rx] = receive()
        
        # Parse distances
        u1_dist = float(responses[0][1])
        u2_dist = float(responses[1][1])
        u4_dist = float(responses[2][1])
        u5_dist = float(responses[3][1])
        
        print(f'Searching... L1:{u1_dist:.1f} L2:{u2_dist:.1f} R4:{u4_dist:.1f} R5:{u5_dist:.1f}')
        
        # Check if BOTH sensors on either side see a wall
        left_both_see_wall = (u1_dist < distance_threshold) and (u2_dist < distance_threshold)
        right_both_see_wall = (u4_dist < distance_threshold) and (u5_dist < distance_threshold)
        
        if left_both_see_wall or right_both_see_wall:
            wall_found = True
            print("✓ Wall found! Both sensors on one side engaged.")
            break
        
        # No wall found yet, keep rotating
        if rotation_count >= max_search_rotations:
            print("ERROR: No wall found after 720° search!")
            return False
        
        print(f"→ Rotating to find wall... ({rotation_count * 10}°)")
        cmd = 'r0:10'
        packet = packetize(cmd)
        transmit(packet)
        responses = receive()
        time.sleep(0.3)
        rotation_count += 1
    
    # === PHASE 2: ALIGNMENT ===
    print("Starting alignment phase...")
    
    while not aligned:
        # Get fresh readings
        cmd = 'u1,u2,u4,u5'
        packet = packetize(cmd)
        transmit(packet)
        [responses, time_rx] = receive()
        
        u1_dist = float(responses[0][1])
        u2_dist = float(responses[1][1])
        u4_dist = float(responses[2][1])
        u5_dist = float(responses[3][1])
        
        # Check which sides have both sensors seeing walls
        left_both_see_wall = (u1_dist < distance_threshold) and (u2_dist < distance_threshold)
        right_both_see_wall = (u4_dist < distance_threshold) and (u5_dist < distance_threshold)
        
        # Calculate differences
        left_diff = abs(u1_dist - u2_dist) if left_both_see_wall else float('inf')
        right_diff = abs(u4_dist - u5_dist) if right_both_see_wall else float('inf')
        
        print(f'Aligning... L_diff:{left_diff:.2f} R_diff:{right_diff:.2f}')
        
        # Determine which side to align to (prefer the one with smaller difference)
        if left_both_see_wall and right_both_see_wall:
            # Both sides visible, choose the more aligned one
            align_to_left = left_diff <= right_diff
        elif left_both_see_wall:
            align_to_left = True
        elif right_both_see_wall:
            align_to_left = False
        else:
            # Lost the wall, restart search
            print("⚠ Wall lost during alignment, restarting search...")
            wall_found = False
            return bootwallalign()  # Recursive call to restart
        
        # Check if aligned
        current_diff = left_diff if align_to_left else right_diff
        
        if current_diff < align_threshold:
            side_name = "LEFT" if align_to_left else "RIGHT"
            print(f"✓ {side_name} SIDE ALIGNED (diff:{current_diff:.2f})")
            print("=== BOOT ALIGNMENT COMPLETE ===")
            aligned = True
            break
        
        # Perform alignment adjustment
        if align_to_left:
            print(f"Aligning to LEFT wall (diff:{left_diff:.2f})")
            if u1_dist > u2_dist:
                # u1 (front) farther, turn CCW
                cmd = 'r0:-5'
            else:
                # u2 (rear) farther, turn CW
                cmd = 'r0:5'
        else:
            print(f"Aligning to RIGHT wall (diff:{right_diff:.2f})")
            if u5_dist > u4_dist:
                # u5 (front) farther, turn CW
                cmd = 'r0:5'
            else:
                # u4 (rear) farther, turn CCW
                cmd = 'r0:-5'
        
        # Execute alignment command
        packet = packetize(cmd)
        transmit(packet)
        responses = receive()
        time.sleep(0.5)
    
    return True

def wallalign(): 
    """Wall align used in between drive commands - prioritizes the side that is more aligned
    Will NOT align if either side has differential < 0.4 inches"""
    
    align_threshold = 0.2
    distance_threshold = 5.0
    misalignment_threshold = 0.4  # Only align if difference > 0.4 inch
    adjust_angle = 5  # degrees per adjustment step
    
    max_attempts = 4       # ← NEW
    attempts = 0           # ← NEW
    
    aligned = False
    
    # Get ultrasonic readings for u1, u2, u4, u5
    cmd = 'u1,u2,u4,u5'
    packet = packetize(cmd)
    transmit(packet)
    [responses, time_rx] = receive()
    
    # Parse distances
    u1_dist = float(responses[0][1])
    u2_dist = float(responses[1][1])
    u4_dist = float(responses[2][1])
    u5_dist = float(responses[3][1])
    
    print(f'u1: {u1_dist}, u2: {u2_dist}, u4: {u4_dist}, u5: {u5_dist}')
    
    # Check if BOTH sensors on each side are within threshold
    left_both_see_wall = (u1_dist < distance_threshold) and (u2_dist < distance_threshold)
    right_both_see_wall = (u4_dist < distance_threshold) and (u5_dist < distance_threshold)
    
    # If NO side sees a wall, nothing to align
    if not left_both_see_wall and not right_both_see_wall:
        print("No walls detected on either side - continuing straight")
        return
    
    # Calculate differences
    left_diff = abs(u1_dist - u2_dist) if left_both_see_wall else float('inf')
    right_diff = abs(u4_dist - u5_dist) if right_both_see_wall else float('inf')
    
    # Skip if already aligned enough
    if left_both_see_wall and left_diff < misalignment_threshold:
        print(f"Wall alignment skipped - left side well aligned (diff: {left_diff:.2f}in < {misalignment_threshold}in)")
        return
    if right_both_see_wall and right_diff < misalignment_threshold:
        print(f"Wall alignment skipped - right side well aligned (diff: {right_diff:.2f}in < {misalignment_threshold}in)")
        return
    
    # Determine whether alignment is needed
    left_needs_align = left_both_see_wall and (left_diff > misalignment_threshold)
    right_needs_align = right_both_see_wall and (right_diff > misalignment_threshold)
    
    if not left_needs_align and not right_needs_align:
        print(f"Wall alignment not needed (L diff: {left_diff:.2f}in, R diff: {right_diff:.2f}in)")
        return
    
    # Choose which wall to align to (prefer the more aligned one)
    align_to_left = left_needs_align and (not right_needs_align or left_diff <= right_diff)
    
    if align_to_left:
        print(f"Aligning to left wall (diff: {left_diff:.2f}in)")
    else:
        print(f"Aligning to right wall (diff: {right_diff:.2f}in)")
    
    # =========================================================
    #  ALIGNMENT LOOP — NOW LIMITED TO 4 ATTEMPTS
    # =========================================================
    while not aligned and attempts < max_attempts:
        
        attempts += 1  # ← count attempt
        
        # Fresh readings
        cmd = 'u1,u2,u4,u5'
        packet = packetize(cmd)
        transmit(packet)
        [responses, time_rx] = receive()
        
        u1_dist = float(responses[0][1])
        u2_dist = float(responses[1][1])
        u4_dist = float(responses[2][1])
        u5_dist = float(responses[3][1])
        
        # LEFT ALIGNMENT
        if align_to_left:
            left_diff = abs(u1_dist - u2_dist)
            
            if left_diff < align_threshold:
                print(f"Left side aligned (diff: {left_diff:.2f}in)")
                aligned = True
                break
            
            # Decide direction
            if u1_dist > u2_dist:
                cmd = f'r0:-{adjust_angle}'   # CCW
            else:
                cmd = f'r0:{adjust_angle}'    # CW
            
            packet = packetize(cmd)
            transmit(packet)
            responses = receive()
            time.sleep(0.3)
            continue
        
        # RIGHT ALIGNMENT
        else:
            right_diff = abs(u4_dist - u5_dist)
            
            if right_diff < align_threshold:
                print(f"Right side aligned (diff: {right_diff:.2f}in)")
                aligned = True
                break
            
            # Decide direction
            if u5_dist > u4_dist:
                cmd = f'r0:{adjust_angle}'     # CW
            else:
                cmd = f'r0:-{adjust_angle}'    # CCW
            
            packet = packetize(cmd)
            transmit(packet)
            responses = receive()
            time.sleep(0.3)
            continue
    
    # =========================================================
    #  END OF ALIGNMENT ATTEMPTS
    # =========================================================
    
    if attempts >= max_attempts and not aligned:
        print(f"Wall alignment stopped after {attempts} attempts (NOT fully aligned).")
    else:
        print("Wall alignment complete")
    
    return


def drive_adjustment():
    """
    Check side sensors and adjust robot orientation before drive command if too close to walls.
    Reads both side sensors and turns away from any wall that's too close.
    If too far from wall (3.5-5 inches), turn back toward it.
    """
    min_distance = 0.1
    max_distance = 1.2
    far_min = 4.1
    far_max = 6.0
    turn_angle = 8  # degrees to turn for adjustment
    
    cmd = 'u1,u2,u4,u5'
    packet = packetize(cmd)
    transmit(packet)
    [responses, time_rx] = receive()
    
    u1_dist = float(responses[0][1])
    u2_dist = float(responses[1][1])
    u4_dist = float(responses[2][1])
    u5_dist = float(responses[3][1])
    
    print(f"Side sensors - L1:{u1_dist:.2f} L2:{u2_dist:.2f} R1:{u4_dist:.2f} R2:{u5_dist:.2f}")
    
    # Check if too close to left wall (turn RIGHT away from wall)
    if min_distance < u1_dist < max_distance or min_distance < u2_dist < max_distance:
        print(f"⚠ Left side too close - turning RIGHT {turn_angle}°")
        adjust_cmd = packetize(f"r0:{turn_angle}")
        transmit(adjust_cmd)
        [responses, time_rx] = receive()
        time.sleep(0.2)
    
    # Check if too close to right wall (turn LEFT away from wall)
    elif min_distance < u4_dist < max_distance or min_distance < u5_dist < max_distance:
        print(f"⚠ Right side too close - turning LEFT {turn_angle}°")
        adjust_cmd = packetize(f"r0:-{turn_angle}")
        transmit(adjust_cmd)
        [responses, time_rx] = receive()
        time.sleep(0.2)
    
    # Check if too far from left wall (turn LEFT toward wall)
    elif far_min < u1_dist < far_max or far_min < u2_dist < far_max:
        print(f"⚠ Left side too far - turning LEFT {turn_angle}° toward wall")
        adjust_cmd = packetize(f"r0:-{turn_angle}")
        transmit(adjust_cmd)
        [responses, time_rx] = receive()
        time.sleep(0.2)
    
    # Check if too far from right wall (turn RIGHT toward wall)
    elif far_min < u4_dist < far_max or far_min < u5_dist < far_max:
        print(f"⚠ Right side too far - turning RIGHT {turn_angle}° toward wall")
        adjust_cmd = packetize(f"r0:{turn_angle}")
        transmit(adjust_cmd)
        [responses, time_rx] = receive()
        time.sleep(0.2)
    
    else:
        print("✓ Side clearance OK - no adjustment needed")
    
    return

def center_to_front_wall(target_distance=4.5, tolerance=0.5, max_attempts=20):
    """
    Center robot to front wall at specified distance.
    
    Args:
        target_distance: Target distance from front wall (inches)
        tolerance: Acceptable error range (inches)
        max_attempts: Maximum number of centering attempts
    
    Returns:
        bool: True if successfully centered, False if timeout or failed
    """
    print(f"Centering to front wall (target: {target_distance}in)...")
    
    centering_attempts = 0
    
    while centering_attempts < max_attempts:
        # Read current front distance
        packet = packetize("u0")
        transmit(packet)
        responses, _ = receive()
        current_front = float(responses[0][1])
        
        error = current_front - target_distance
        
        print(f"  Attempt {centering_attempts + 1}: Front={current_front:.2f}in | Error={error:+.2f}in")
        
        # Check if centered
        if abs(error) < tolerance:
            print(f"✓ Centered at {current_front:.2f}in")
            return True
        
        # Wait for robot to be ready
        packet_ready = packetize('w0')
        transmit(packet_ready)
        ready_resp, _ = receive()
        
        if ready_resp[0][1] == 'True':
            # Move forward or backward based on error
            if error > tolerance:
                # Too far, move forward 0.5 inch
                move_cmd = packetize("w0:0.5")
                print(f"  → Moving forward 0.5in")
            elif error < -tolerance:
                # Too close, move backward 1 inch
                move_cmd = packetize("w0:-1")
                print(f"  → Moving backward 1in")
            
            transmit(move_cmd)
            move_resp, _ = receive()
            
            # Wait for movement to complete
            while True:
                packet_check = packetize('w0')
                transmit(packet_check)
                check_resp, _ = receive()
                if check_resp[0][1] == 'True':
                    break
                time.sleep(0.1)
            
            time.sleep(0.2)  # Small buffer after completion
        
        centering_attempts += 1
    
    print(f"⚠ Centering timeout after {max_attempts} attempts")
    return False

def back_sensor_adjustment():
    """
    Adjust robot position based on back sensor (u3) reading.
    Centers robot to either 14 or 26 inches from back wall.
    
    Targets:
        - If u3 is within 11-17 inches → center to 14 inches
        - If u3 is within 23-29 inches → center to 26 inches
    
    Tolerance: 0.5 inches
    """
    target_14 = 14.0
    target_26 = 25.0
    tolerance = 0.5
    max_attempts = 20
    
    # Read back sensor
    packet = packetize("u3")
    transmit(packet)
    responses, _ = receive()
    u3_dist = float(responses[0][1])
    
    print(f"Back sensor (u3): {u3_dist:.2f}in")
    
    # Determine if adjustment is needed and which target
    target = None
    
    if target_14 - 3 <= u3_dist <= target_14 + 3:
        target = target_14
        print(f"→ Back sensor in 14-inch range ({u3_dist:.2f}in), centering to {target}in")
    elif target_26 - 3 <= u3_dist <= target_26 + 10:
        target = target_26
        print(f"→ Back sensor in 26-inch range ({u3_dist:.2f}in), centering to {target}in")
    else:
        print(f"✓ Back sensor outside adjustment ranges - no centering needed")
        return False
    
    # Perform centering
    attempts = 0
    
    while attempts < max_attempts:
        # Read current back distance
        packet = packetize("u3")
        transmit(packet)
        responses, _ = receive()
        current_back = float(responses[0][1])
        
        error = current_back - target
        
        print(f"  Attempt {attempts + 1}: Back={current_back:.2f}in | Target={target:.2f}in | Error={error:+.2f}in")
        
        # Check if centered
        if abs(error) < tolerance:
            print(f"✓ Centered to {target}in (current: {current_back:.2f}in)")
            return True
        
        # Wait for robot to be ready
        packet_ready = packetize('w0')
        transmit(packet_ready)
        ready_resp, _ = receive()
        
        if ready_resp[0][1] == 'True':
            # Move forward or backward based on error
            # Positive error = too far from wall = move backward
            # Negative error = too close to wall = move forward
            if error > tolerance:
                # Too far, move backward 1 inch
                move_cmd = packetize("w0:-1")
                print(f"  → Moving backward 1in")
            elif error < -tolerance:
                # Too close, move forward 0.5 inch
                move_cmd = packetize("w0:0.5")
                print(f"  → Moving forward 0.5in")
            
            transmit(move_cmd)
            move_resp, _ = receive()
            
            # Wait for movement to complete
            while True:
                packet_check = packetize('w0')
                transmit(packet_check)
                check_resp, _ = receive()
                if check_resp[0][1] == 'True':
                    break
                time.sleep(0.1)
            #realign after each adjustment
            wallalign()
            
            time.sleep(0.2)  # Small buffer after completion
        
        attempts += 1
    
    print(f"⚠ Back sensor centering timeout after {max_attempts} attempts")
    return False

if __name__ == "__main__":
    print ("Testing wall alignment function")
    bootwallalign()
    
