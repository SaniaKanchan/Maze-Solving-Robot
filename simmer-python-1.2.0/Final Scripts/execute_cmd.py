from avoidance import wallalign, bootwallalign, center_to_front_wall, drive_adjustment, SOURCE
from comms import transmit, receive, packetize
from astar import astar, visualize_maze, path_to_commands
import time


def boot_and_align():
    """Perform boot wall alignment sequence
    Then move backwards until front sensor reads ~2.5 inches"""
    center_dist = 3.0
    threshold = 0.5
    wall_threshold = 7.0
    centered = False
    print("\n--- Boot wall alignment sequence ---")
    bootwallalign()
    # to where the front reads the largest distance
    packet = packetize("u0,u2,u3,u4")
    transmit(packet)
    responses, _ = receive()
    front_dist = float(responses[0][1])
    left_dist = float(responses[1][1])
    back_dist = float(responses[2][1])
    right_dist = float(responses[3][1])
    #rotate to the open side if wall is in the front or if the back distance is larger than the front
    if front_dist < wall_threshold or front_dist < back_dist:
        if back_dist > wall_threshold:
            print("Rotating 180 to find open space")
            cmd = "r0:180"  # rotate 180    
        elif left_dist > right_dist:
            print("Rotating left to find open space")
            cmd = "r0:-90"  # rotate left
        else: 
            print("Rotating right to find open space")
            cmd = "r0:90"  # rotate right
        packet = packetize(cmd)
        transmit(packet)
        responses, _ = receive()
        # wait for command to complete
        while True:
            packet_check = packetize('w0')
            transmit(packet_check)
            check_resp, _ = receive()
            if check_resp[0][1] == 'True':
                break
    # Move backwards until 2.5 in from wall
    while not centered:
        packet = packetize("u3")
        transmit(packet)
        responses, _ = receive()
        back_dist = float(responses[0][1])
        #move in larger steps if far away, smaller if close
        if back_dist > 5:
            cmd = "w0:-3"  # move backward
            transmit(packetize(cmd))
            responses, _ = receive()
            # wait for command to complete
            while True:
                packet_check = packetize('w0')
                transmit(packet_check)
                check_resp, _ = receive()
                if check_resp[0][1] == 'True':
                    break
        elif 0.2 < back_dist < center_dist - threshold:
            cmd = "w0:1"  # move forward
            packet = packetize(cmd)
            transmit(packet)
            responses, _ = receive()
            # wait for command to complete
            while True:
                packet_check = packetize('w0')
                transmit(packet_check)
                check_resp, _ = receive()
                if check_resp[0][1] == 'True':
                    break
        elif back_dist > center_dist + threshold:
            
            cmd = "w0:-1"  # move backward
            packet = packetize(cmd)
            transmit(packet)
            responses, _ = receive()
            # wait for command to complete
            while True:
                packet_check = packetize('w0')
                transmit(packet_check)
                check_resp, _ = receive()
                if check_resp[0][1] == 'True':
                    break
        else:
            centered = True
           
    

def execute_cmds_with_safety(cmds):
    """
    Execute a list of drive commands with front sensor safety checks.
    Skips forward commands if obstacle is detected and jumps to next rotation.
    Handles optional centering before rotations and waits for drive readiness.
    Runs wall align AFTER each command execution.
    """
    i = 0
    while i < len(cmds):
        cmd = cmds[i]

        # ---------------- FRONT SENSOR CHECK ----------------
        packet = packetize("u0")
        transmit(packet)
        responses, _ = receive()
        front_dist = float(responses[0][1])

        # ---------------- OBSTACLE DETECTION ----------------
        if cmd.startswith("w0") and 0.5 < front_dist < 8:
            print("⚠ OBSTACLE DETECTED! Skipping forward commands...")

            # Center robot to front wall before rotation
            center_to_front_wall()

            # Find next rotation command
            next_rot = i
            while next_rot < len(cmds) and cmds[next_rot].startswith("w0"):
                next_rot += 1

            if next_rot >= len(cmds):
                print("✗ No rotation command left after obstacle. Stopping.")
                break

            # Jump to rotation command
            print(f"→ Jumping to rotation: {cmds[next_rot]}")
            i = next_rot
            cmd = cmds[i]

        # ---------------- CENTER BEFORE ROTATION ----------------
        if cmd.startswith("r0"):
            packet = packetize("u0")
            transmit(packet)
            responses, _ = receive()
            front_dist = float(responses[0][1])
            print(f"Before turn - Front sensor: {front_dist} in")

            if 0.5 < front_dist < 10:
                center_to_front_wall()
            else:
                print("→ No front wall detected (<0.5 or >10 in), skip centering")

        # ---------------- WAIT FOR DRIVE SYSTEM ----------------
        command_sent = False
        while not command_sent:
            packet_w0 = packetize('w0')
            transmit(packet_w0)
            responses, _ = receive()
            status = responses[0][1]

            if status == 'True':
                # Pre-drive adjustment ONLY for forward moves
                if cmd.startswith("w0:"):
                    print("\n--- Pre-drive adjustment ---")
                    drive_adjustment()
                    print("✓ Drive adjustment complete\n")

                # Send actual command
                packet_cmd = packetize(cmd)
                transmit(packet_cmd)
                responses, _ = receive()
                cmd_status = responses[0][1]

                if cmd_status == 'True':
                    command_sent = True
                    # Wait for command completion
                    print("→ Waiting for command to complete...")
                    while True:
                        packet_check = packetize('w0')
                        transmit(packet_check)
                        check_resp, _ = receive()
                        if check_resp[0][1] == 'True':
                            print(f"✓ Command '{cmd}' completed")
                            break
                        time.sleep(0.1)
                    
                    # ---------------- POST-COMMAND WALL ALIGN ----------------
                    # Run wall align AFTER both rotation and drive commands
                    print("\n--- Post-command wall alignment ---")
                    wallalign()
                    print("✓ Wall align complete\n")
                    
                else:
                    time.sleep(0.01)  # retry

            else:
                time.sleep(0.01)  # wait if busy

        # ---------------- NEXT COMMAND ----------------
        i += 1
        
# test boot_and_align
if __name__ == "__main__":
    boot_and_align()
