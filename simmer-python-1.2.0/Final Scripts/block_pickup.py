from comms import transmit, receive, packetize, SOURCE
import time
from avoidance import wallalign, bootwallalign, center_to_front_wall


# ============================================================
#  SECTION 3 – ROBOT HELPERS (MODIFIED)
# ============================================================
def wait_until_ready():
    """Poll w0 until the drive controller reports True."""
    while True:
        pkt = packetize("w0")
        transmit(pkt)
        resp, _ = receive()
        try:
            if resp[0][1] == "True":
                break
        except:
            pass
        time.sleep(0.1)

def send_drive(cmd: str) -> bool:
    pkt = packetize(cmd)
    if not pkt:
        return False

    transmit(pkt)
    resp, _ = receive()

    try:
        name = cmd.split(":")[0]
        return (resp[0][0] == name and resp[0][1] == "True")
    except:
        return False


def read_u0():
    pkt = packetize("u0")
    transmit(pkt)
    resp, _ = receive()
    try:
        if resp[0][0] == "u0":
            return float(resp[0][1])
    except:
        return None
    return None


def read_ub():
    pkt = packetize("ub")
    transmit(pkt)
    resp, _ = receive()
    try:
        if resp[0][0] == "ub":
            return float(resp[0][1])
    except:
        return None
    return None


# ---- Track heading ----
current_heading = 0.0

def rotate(deg):
    global current_heading
    
    # FIX: Round and cast to int to ensure command is "r0:-30", not "r0:-30.0"
    deg_int = int(round(deg))
    cmd = f"r0:{deg_int}"
    pkt = packetize(cmd)
    transmit(pkt)

    # *** WAIT FOR ROBOT'S RESPONSE ***
    resp, _ = receive()

    # FIX: Increased dynamic sleep based on turn size. 
    # Old: 0.10 + (deg * 0.04)
    # New: 0.20 + (deg * 0.05) for extremely safe timing
    sleep_time = 0.20 + (abs(deg_int) * 0.05)
    
    # Clamp minimum sleep increased to 0.30
    if sleep_time < 0.30: sleep_time = 0.30
        
    time.sleep(sleep_time)
    
    current_heading = current_heading + deg_int


def drive_forward(inch):
    send_drive(f"w0:{inch}")
    time.sleep(0.2)


def face_north(rotate):
    global current_heading
    
    # If we are already close enough, do nothing
    if abs(current_heading) < 1.0:
        current_heading = 0.0
        return

    print(f"[FACE NORTH] Stepping to 0 from {current_heading:.0f}°")
    
    target_angle = 0.0
    step_size = 5 # Enforced 5 degree step
    
    # Standard stable stepping loop (prevents oscillation)
    while abs(target_angle - current_heading) > 0.5:
        diff = target_angle - current_heading
        
        if abs(diff) > step_size:
            step = step_size if diff > 0 else -step_size
        else:
            step = diff
            
        rotate(step)
    
    # Force variable to 0.0
    current_heading = 0.0
    
def go_to_top_wall(rotate, read_u0, drive_forward):
    print("=== GOING TO TOP WALL ===")
    face_north(rotate)

    while True:
        d = read_u0()
        if d is None: d = 999
        if d <= 7:
            break
        drive_forward(1)
        time.sleep(0.1)
    center_to_front_wall()
    rotate(90) # Face East

    # Align before the long drive
    wallalign()
    time.sleep(0.5) # Let it settle

# ============================================================
#  SECTION 4 – LOGIC FUNCTIONS (FIXED)
# ============================================================


# UPDATE: Added step_size parameter to accept values from mini_scan
def bump_scan_dual(start_angle, end_angle, rotate, read_u0, read_ub, step_size=10):
    global current_heading
    STEP = step_size # Use the passed step size (defaults to 10)
    DIFF_THRESHOLD = 5
    MAX_BOT_DIST = 12.0  # ub must be less than 10 inches
    
    # Direction of scan
    step_val = STEP if end_angle >= start_angle else -STEP

    print(f"\n=== BUMP SCAN {start_angle:.0f}° -> {end_angle:.0f}° (Step: {step_size}) ===")

    # 1. NAVIGATE TO START ANGLE (INCREMENTAL)
    # UPDATE: This now uses a loop to ensure even the setup turn is stepped by 8
    target_angle = start_angle
    print(f"[SAFE ROTATE] Stepping to Start: {current_heading:.0f}° -> {target_angle:.0f}°")

    while abs(target_angle - current_heading) > 0.5:
        diff = target_angle - current_heading
        if abs(diff) > step_size:
            step = step_size if diff > 0 else -step_size
        else:
            step = diff
        rotate(step)
        wait_until_ready() # Ensure robot is ready before next step
    
    # Force-sample to clear old data
    read_u0(); time.sleep(0.1)
    read_u0()

    # 2. SCAN LOOP
    hit_angles = []
    consecutive_misses = 0 
    
    scan_range = range(int(start_angle), int(end_angle) + step_val, step_val)

    for target_angle in scan_range:
        step_diff = target_angle - current_heading
        rotate(step_diff)
        time.sleep(0.5)

        bot_d = read_ub(); time.sleep(0.06) 
        top_d = read_u0(); time.sleep(0.06)

        if top_d is None: top_d = 999
        if bot_d is None: bot_d = 999
        diff = top_d - bot_d  # Changed to directional difference (u0 - ub)

        # New condition: u0 > ub by at least 6, and ub < 10
        if 0.1 <bot_d < MAX_BOT_DIST and diff >= DIFF_THRESHOLD:
            print(f"[SCAN] {current_heading:4.0f}° | u0={top_d:6.2f} | ub={bot_d:6.2f} | diff={diff:6.2f} *HIT*")
            hit_angles.append(current_heading)
            consecutive_misses = 0 
        else:
            print(f"[SCAN] {current_heading:4.0f}° | u0={top_d:6.2f} | ub={bot_d:6.2f} | diff={diff:6.2f}")
            if len(hit_angles) >= 2:
                consecutive_misses += 1
                if consecutive_misses >= 3:
                    print(">>> Object passed (3 consecutive misses). Stopping scan early.")
                    break

    # 3. PROCESS RESULTS
    if not hit_angles:
        print(">>> No block found.")
        return False, current_heading

    clusters = []
    current_group = [hit_angles[0]]

    for i in range(1, len(hit_angles)):
        if abs(hit_angles[i] - hit_angles[i-1]) <= abs(STEP) + 1:
            current_group.append(hit_angles[i])
        else:
            clusters.append(current_group)
            current_group = [hit_angles[i]]
    clusters.append(current_group)

    clusters.sort(key=len, reverse=True)
    best_group = clusters[0]
    center_angle = sum(best_group) / len(best_group)

    print(f">>> TARGET ANGLE: {center_angle:.2f}°")
    return center_angle, current_heading


def drive_to_pickup(block_angle, rotate, read_ub, read_u0, drive_forward, transmit):
    global current_heading 

    # -------------------------------------------------
    # 1. ROTATE TO BLOCK (INCREMENTAL)
    # -------------------------------------------------
    target_angle = block_angle
    step_size = 9
    
    print(f"[DRIVE] Stepping to Block: {current_heading:.0f}° -> {target_angle:.0f}°")

    while abs(target_angle - current_heading) > 0.5:
        diff = target_angle - current_heading
        if abs(diff) > step_size:
            step = step_size if diff > 0 else -step_size
        else:
            step = diff
        rotate(step)

    time.sleep(.8)

    # -------------------------------------------------
    # 2. APPROACH LOOP
    # -------------------------------------------------
    while True:
        ub = read_ub(); time.sleep(0.05)
        u0 = read_u0()
        
        if ub is None: ub = 999
        if u0 is None: u0 = 999
        diff = abs(ub - u0)
        
        print(f"[DRIVE] ub={ub:.1f}, u0={u0:.1f}, diff={diff:.1f}")

        # SUCCESS
        if ub <= 1.1:
            print(">>> ARRIVED AT BLOCK.")
            break

        # LOST SIGNAL / AMBIGUOUS
        if diff <= 6.0: 
            print(">>> Signal Ambiguous. BLIND FORCE...")
            drive_forward(2) 
            time.sleep(1.0) 
            drive_forward(2) 
            time.sleep(1.0) 

            check_ub = read_ub()
            if check_ub is not None and check_ub <= 2.0:
                print(">>> Blind success.")
                break
            else:
                print(">>> LOST1. Backing up.")
                drive_forward(-1)
                time.sleep(0.5)
                
                # --- NEW RECOVERY LOGIC ---
                # Scan from WHERE WE ARE (current_heading)
                # To +40 degrees (Right)
                # Using 10 degree steps
                
                rel_start = current_heading
                rel_end = current_heading + 40
                
                print(f">>> Rescanning Right: {rel_start:.0f}° to {rel_end:.0f}° (Step: 10)")
                
                return mini_scan(rotate, read_u0, read_ub, drive_forward, transmit, 
                               start_angle=rel_start, end_angle=rel_end, scan_step=10)

        # STANDARD DRIVE
        drive_forward(2)
        time.sleep(0.1)

    # -------------------------------------------------
    # 3. PICKUP
    # -------------------------------------------------
    time.sleep(0.5)
    drive_forward(1) 
    
    print(">>> Sending Block Pickup (bp) command...")
    pkt = packetize("bp")
    transmit(pkt)
    
    print(">>> Waiting 5s for arm actuation...")
    time.sleep(5) 
    print(">>> Pickup sequence complete.")

    print(">>> Initiating return to Top Wall...")
    go_to_top_wall(rotate, read_u0, drive_forward)
    return True

# UPDATE: Added scan_step parameter to fix the crash
def mini_scan(rotate, read_u0, read_ub, drive_forward, transmit, start_angle=-20, end_angle=35, scan_step=10):
    global current_heading
    print(f"\n=== MINI SCAN ({start_angle:.0f} to {end_angle:.0f}) ===")
    #first move forward 4in to get better readings
    drive_forward(2)
    # Pass the scan_step to bump_scan_dual
    angle, _ = bump_scan_dual(start_angle, end_angle, rotate, read_u0, read_ub, step_size=scan_step)

    if angle is False:
        print("MINI SCAN -> No block -> Resetting N")
        face_north(rotate)
        wallalign()
        return False

    print(f"MINI SCAN -> Found at {angle:.2f}°")
    return drive_to_pickup(angle, rotate, read_ub, read_u0, drive_forward, transmit)


def full_scan(rotate, read_u0, read_ub, drive_forward, transmit):
    global current_heading

    start_angle = -20
    end_angle = 120

    while True:
        print("\n=== FULL SCAN ===")

        # For full scan, starting at a known zero is helpful
        face_north(rotate)
        wallalign()
        
        # Added sleep to ensure "reset to 0" completes before "rotate to -30" is sent
        time.sleep(0.5)

        angle, _ = bump_scan_dual(start_angle, end_angle, rotate, read_u0, read_ub)

        if angle is not False:
            print(f"FULL SCAN -> Block detected at {angle:.2f}° -> PICKUP")
            return drive_to_pickup(angle, rotate, read_ub, read_u0, drive_forward, transmit)

        # No block found
        print("FULL SCAN -> No block found. Resetting N and moving forward...")
        face_north(rotate)
        drive_forward(2)
        time.sleep(0.3)

def ilz(rotate, read_u0, read_ub, drive_forward, transmit):

    print("=== ILZ START ===")

    # MINI SCAN
    print("[ILZ] Running mini scan...")
    # Default values used (-10 to 25)
    found = mini_scan(rotate, read_u0, read_ub, drive_forward, transmit)

    if found:
        print("[ILZ] Block found by mini_scan. Going to top wall...")
        return True

    # Align before the long drive
    wallalign() 
    
    drive_forward(9)
    time.sleep(3)

    # FULL SCAN LOOP
    while True:
        print("[ILZ] Running full scan...")
        found = full_scan(rotate, read_u0, read_ub, drive_forward, transmit)

        if found:
            print("[ILZ] Block found by full_scan. Success.")
            return True

        print("[ILZ] No block in full scan. Moving forward 2 inches and retrying...")
        drive_forward(2)
        time.sleep(0.5)

# ============================================================
#  SECTION 5 – MAIN CLIENT
# ============================================================

def block_scan_pickup():
    print(f"\n========== CLIENT ({SOURCE}) ==========\n")
    
    while True:
        cmd = input("Type command (ilz / miniscan / fullscan / topwall / exit): ").strip()

        if cmd in ("exit", "quit"):
            print("Exiting client...")
            break

        if cmd == "miniscan":
            mini_scan(rotate, read_u0, read_ub, drive_forward, transmit)
            continue

        if cmd == "fullscan":
            full_scan(rotate, read_u0, read_ub, drive_forward, transmit)
            continue

        if cmd == "ilz":
            ilz(rotate, read_u0, read_ub, drive_forward, transmit)
            continue
        if cmd == "bd":
            send_drive("bd")
            continue

        # Raw Commands
        pkt = packetize(cmd)
        if not pkt:
            print("Invalid packet format.")
            continue

        transmit(pkt)
        resp, ts = receive()
        print(f"[{ts}] {resp}")

        
if __name__ == "__main__":
    block_scan_pickup()