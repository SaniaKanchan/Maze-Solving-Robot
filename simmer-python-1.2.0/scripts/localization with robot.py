"""
Localization module for maze navigation using wall sensors.

This implements FEATURE-BASED LOCALIZATION (also called Wall Pattern Matching
or Signature-Based Localization), a type of Monte Carlo Localization.

Method:
1. Read ultrasonic sensors to detect walls in all 4 cardinal directions
2. Generate a "wall signature" (binary pattern of which sides have walls)
3. Compare this signature against all known locations in the maze
4. Use movement and multiple readings to disambiguate between candidate positions
5. Converge to the true position and orientation

Key advantages:
- Works with known static environments (like this fixed maze)
- No odometry required for initial localization
- Robust to sensor noise through multiple sampling
- Can recover from complete position uncertainty

This is different from:
- Dead reckoning: Uses only odometry (accumulates error)
- Particle filter MCL: Maintains probability distribution over many particles
- EKF localization: Uses Kalman filtering with Gaussian assumptions
"""

import time

# Import communication functions from main script
# Assumes these are available: transmit, receive, packetize, MAZE, ROWS, COLS

class LocalizationSystem:
    """
    Localizes the rover in a known maze using wall sensor readings.
    """
    
    def __init__(self, maze, rows, cols):
        """
        Initialize localization system.
        
        Args:
            maze: 2D list representing the maze (1=obstacle, 0=free)
            rows: Number of rows in maze
            cols: Number of columns in maze
        """
        self.maze = maze
        self.rows = rows
        self.cols = cols
        
        # Direction mappings
        self.directions = {
            0: 'right',    # East
            90: 'down',    # South
            180: 'left',   # West
            270: 'up'      # North
        }
        
        # Delta movements for each direction (dx, dy)
        self.direction_deltas = {
            0: (1, 0),      # right: +x
            90: (0, 1),     # down: +y
            180: (-1, 0),   # left: -x
            270: (0, -1)    # up: -y
        }
        
    def read_wall_sensors(self):
        """
        Read wall sensor data from the rover using all 6 ultrasonic sensors.
        
        Sensor layout (based on config.py):
            u0: Front middle (0°)
            u1: Left front (90°)
            u2: Left back (90°)
            u3: Back middle (180°)
            u4: Right back (-90°)
            u5: Right front (-90°)
        
        Returns:
            dict: Wall presence in each direction relative to rover
                 {'front': bool, 'right': bool, 'back': bool, 'left': bool}
                 True = wall detected, False = no wall
        """
        # Send individual commands for each sensor
        values = [None] * 6  # to store u0–u5 results
        
        for i in range(6):
            cmd = f"u{i}"
            try:
                packet = globals()['packetize'](cmd)
                if not packet:
                    print(f"ERROR: Failed to packetize sensor command u{i}")
                    continue
                globals()['transmit'](packet)
                time.sleep(0.1)  # Short delay between commands
                responses, _ = globals()['receive']()
                if responses and len(responses) > 0 and len(responses[0]) > 1:
                    try:
                        value = float(responses[0][1])
                        values[i] = value
                        print(f"  Sensor u{i}: {value:.1f} inches")
                    except (ValueError, IndexError):
                        print(f"ERROR: Invalid response for sensor u{i}")
                else:
                    print(f"ERROR: No response for sensor u{i}")
            except Exception as e:
                print(f"ERROR reading sensor u{i}: {e}")
        
        print(f"  All sensor values: {values}")
        
        WALL_THRESHOLD = 5  # inches - adjust based on your setup
        MIN_DISTANCE = 0.5  # Minimum valid distance reading
        
        if not all(v is not None for v in values):
            print("ERROR: Not all sensors returned valid readings")
            return None
        
        # Extract distances from all sensors
        u0_dist = values[0]  # front
        u1_dist = values[1]  # left front
        u2_dist = values[2]  # left back
        u3_dist = values[3]  # back
        u4_dist = values[4]  # right back
        u5_dist = values[5]  # right front
        
        # Aggregate readings for each cardinal direction
        walls = {
            'front': MIN_DISTANCE < u0_dist < WALL_THRESHOLD,
            'right': MIN_DISTANCE < (u4_dist + u5_dist) / 2 < WALL_THRESHOLD,
            'back': MIN_DISTANCE < u3_dist < WALL_THRESHOLD,
            'left': MIN_DISTANCE < (u1_dist + u2_dist) / 2 < WALL_THRESHOLD
        }
        
        # Debug output
        print(f"    Sensor distances: F={u0_dist:.1f} R={u4_dist:.1f}/{u5_dist:.1f} B={u3_dist:.1f} L={u1_dist:.1f}/{u2_dist:.1f}")
        print(f"    Wall detection: {walls}")
        
        return walls
    
    def match_wall_pattern(self, measured_walls):
        """
        Find all possible positions and orientations matching the wall pattern.
        Args:
            measured_walls: Absolute wall readings
        Returns:
            list: List of tuples (x, y, orientation) matching the pattern
        """
        candidates = []
        for y in range(self.rows):
            for x in range(self.cols):
                if self.maze[y][x] == 1:
                    continue
                expected_walls = self.check_walls_at_position(x, y)
                if expected_walls == measured_walls:
                    for orientation in [0, 90, 180, 270]:
                        candidates.append((x, y, orientation))
        return candidates
    
    def get_absolute_walls(self, relative_walls, orientation):
        """
        Convert relative wall readings to absolute maze directions.
        
        Args:
            relative_walls: dict with 'front', 'right', 'back', 'left'
            orientation: Current orientation in degrees (0, 90, 180, 270)
        
        Returns:
            dict: Walls in absolute directions {'north': bool, 'east': bool, 
                                               'south': bool, 'west': bool}
        """
        # Map relative to absolute based on orientation
        # If facing right (0°): front=east, right=south, back=west, left=north
        # If facing down (90°): front=south, right=west, back=north, left=east
        # etc.
        
        rotation_map = {
            0: {'front': 'east', 'right': 'south', 'back': 'west', 'left': 'north'},
            90: {'front': 'south', 'right': 'west', 'back': 'north', 'left': 'east'},
            180: {'front': 'west', 'right': 'north', 'back': 'east', 'left': 'south'},
            270: {'front': 'north', 'right': 'east', 'back': 'south', 'left': 'west'}
        }
        
        mapping = rotation_map[orientation]
        absolute_walls = {}
        
        for rel_dir, abs_dir in mapping.items():
            absolute_walls[abs_dir] = relative_walls[rel_dir]
        
        return absolute_walls
    
    def check_walls_at_position(self, x, y):
        """
        Calculate expected walls at a given position based on maze structure.
        
        Args:
            x: Column index
            y: Row index
        
        Returns:
            dict: Expected walls {'north': bool, 'east': bool, 'south': bool, 'west': bool}
        """
        walls = {
            'north': False,  # up
            'east': False,   # right
            'south': False,  # down
            'west': False    # left
        }
        
        # Check maze boundaries
        if y == 0:
            walls['north'] = True
        if y == self.rows - 1:
            walls['south'] = True
        if x == 0:
            walls['west'] = True
        if x == self.cols - 1:
            walls['east'] = True
        
        # Check adjacent cells for obstacles
        if y > 0 and self.maze[y-1][x] == 1:
            walls['north'] = True
        if y < self.rows - 1 and self.maze[y+1][x] == 1:
            walls['south'] = True
        if x > 0 and self.maze[y][x-1] == 1:
            walls['west'] = True
        if x < self.cols - 1 and self.maze[y][x+1] == 1:
            walls['east'] = True
        
        return walls
    
    def can_move_forward(self, x, y, orientation):
        """
        Check if the rover can move forward from a given position and orientation.
        
        Args:
            x: Column index
            y: Row index
            orientation: Orientation in degrees
        
        Returns:
            bool: True if can move forward, False if blocked
        """
        dx, dy = self.direction_deltas[orientation]
        new_x, new_y = x + dx, y + dy
        
        # Check boundaries
        if new_x < 0 or new_x >= self.cols or new_y < 0 or new_y >= self.rows:
            return False
        
        # Check if target cell is obstacle
        if self.maze[new_y][new_x] == 1:
            return False
        
        return True
    
    def find_turn_direction(self, relative_walls):
        """
        Decide which direction to turn based on wall sensors.
        Priority: right > left > back
        
        Args:
            relative_walls: dict with 'front', 'right', 'back', 'left'
        
        Returns:
            str: 'right', 'left', or 'back'
        """
        if not relative_walls['right']:
            return 'right'
        elif not relative_walls['left']:
            return 'left'
        else:
            return 'back'
    
    def execute_turn(self, direction):
        """
        Execute a turn command.
        
        Args:
            direction: 'right', 'left', or 'back'
        
        Returns:
            int: Degrees turned (90, -90, or 180)
        """
        turn_commands = {
            'right': ('r0:90', 90),
            'left': ('r0:-90', -90),
            'back': ('r0:180', 180)
        }
        
        cmd, degrees = turn_commands[direction]
        print(f"  Turning {direction} ({degrees}°)...")
        
        packet = globals()['packetize'](cmd)
        if packet:
            globals()['transmit'](packet)
            time.sleep(2)  # Allow time for turn
            globals()['receive']()  # Wait for completion
        
        return degrees
    
    def update_candidates_after_turn(self, candidates, turn_degrees):
        """
        Update candidate positions after a turn.
        Position stays same, orientation changes.
        
        Args:
            candidates: List of (x, y, orientation) tuples
            turn_degrees: Degrees turned (positive = clockwise)
        
        Returns:
            list: Updated candidates with new orientations
        """
        updated = []
        for x, y, orientation in candidates:
            new_orientation = (orientation + turn_degrees) % 360
            updated.append((x, y, new_orientation))
        return updated
    
    def update_candidates_after_move(self, candidates):
        """
        Update candidate positions after moving forward.
        
        Args:
            candidates: List of (x, y, orientation) tuples
        
        Returns:
            list: Updated candidates, filtered to valid positions only
        """
        updated = []
        for x, y, orientation in candidates:
            # Check if this candidate can move forward
            if self.can_move_forward(x, y, orientation):
                dx, dy = self.direction_deltas[orientation]
                new_x, new_y = x + dx, y + dy
                updated.append((new_x, new_y, orientation))
            # Also keep original position (might still be in same cell)
            updated.append((x, y, orientation))
        
        return list(set(updated))  # Remove duplicates
    
    def visualize_wall_signature(self, walls, x=None, y=None):
        """
        Print a visual representation of wall signature.
        Args:
            walls: Wall dictionary with 'north', 'east', 'south', 'west' keys
            x: Optional x position for labeling
            y: Optional y position for labeling
        """
        n = '█' if walls.get('north', False) else ' '
        e = '█' if walls.get('east', False) else ' '
        s = '█' if walls.get('south', False) else ' '
        w = '█' if walls.get('west', False) else ' '
        pos_label = f"({x},{y})" if x is not None and y is not None else ""
        print(f"    Wall Signature {pos_label}:")
        print(f"       {n}")
        print(f"      {w}·{e}")
        print(f"       {s}")
    
    def localize_with_movement(self, max_samples=50):
        """
        Perform localization by taking multiple sensor readings while moving.
        Turns when blocked by walls instead of going through them.
        Continues sampling until only 1 configuration remains or max_samples is reached.
        
        Args:
            max_samples: Maximum number of movement steps to take for localization
        
        Returns:
            tuple: (x, y, orientation) or None if localization fails
        """
        print("\n" + "="*60)
        print("Starting Localization Sequence")
        print("="*60)
        
        # Store all possible positions for each reading
        all_candidates = []
        
        for sample in range(max_samples):
            print(f"\nSample {sample + 1}")
            
            # Allow user to take sensor readings
            while True:
                user_input = input("  Type 's' to enter sensor data, 'p' to use data for localization, 'm' to proceed to movement: ").strip().lower()
                
                if user_input == 's':
                    print("  Enter wall detection (true/false for front,right,back,left):")
                    try:
                        front = input("    Front wall? (y/n): ").strip().lower() == 'y'
                        right = input("    Right wall? (y/n): ").strip().lower() == 'y'
                        back = input("    Back wall? (y/n): ").strip().lower() == 'y'
                        left = input("    Left wall? (y/n): ").strip().lower() == 'y'
                        relative_walls = {
                            'front': front,
                            'right': right,
                            'back': back,
                            'left': left
                        }
                        print(f"  Manual sensor reading: {relative_walls}")
                        last_reading = relative_walls
                    except ValueError:
                        print("  Invalid input, try again")
                        
                elif user_input == 'p':
                    if 'last_reading' in locals():
                        relative_walls = last_reading
                        print(f"  Using sensor reading: {relative_walls}")
                        break
                    else:
                        print("  No sensor reading available. Type 's' first.")
                        
                elif user_input == 'm':
                    print("  Proceeding to movement without new reading")
                    continue
                    
                else:
                    print("  Invalid command. Use 's', 'p', or 'm'")
            
            # Process the sensor reading for localization
            print(f"  Relative walls: {relative_walls}")
            
            # Try all possible orientations and find matches
            sample_candidates = []
            
            for orientation in [0, 90, 180, 270]:
                absolute_walls = self.get_absolute_walls(relative_walls, orientation)
                matches = self.match_wall_pattern(absolute_walls)
                
                # Filter matches to only include this orientation
                orientation_matches = [(x, y, o) for x, y, o in matches if o == orientation]
                sample_candidates.extend(orientation_matches)
            
            print(f"  Found {len(sample_candidates)} candidate positions")
            all_candidates.append(sample_candidates)
            
            # Find common candidates across all samples so far
            if len(all_candidates) > 1:
                common = self.intersect_candidate_history(all_candidates)
                print(f"  Consistent positions remaining: {len(common)}")
                
                # Check if we have converged to a single configuration
                if len(common) == 1:
                    print("  Converged to single configuration!")
                    break
            else:
                common = set(sample_candidates)
            
            # Decide next action (unless this is the last sample)
            if sample < max_samples - 1:
                command = input("  Type your movement command (e.g., 'f', 'r', 'l', 'b', 'c', or any custom command): ").strip().lower()

                if command:
                    print(f"  Executing command: {command}")
                    # Send the command
                    packet = globals()['packetize'](command)
                    if packet:
                        globals()['transmit'](packet)
                        time.sleep(1)  # Brief wait for command execution
                        globals()['receive']()  # Wait for completion
                    else:
                        print("  Failed to packetize command")
                        continue

                    # Update all candidate histories based on movement type
                    if command.startswith('w') or command == 'f':
                        # Forward movement
                        for i in range(len(all_candidates)):
                            all_candidates[i] = self.update_candidates_after_move(
                                all_candidates[i]
                            )
                    elif command.startswith('r') or command in ['r', 'l', 'b']:
                        # Rotation - determine degrees based on command
                        if command == 'r':
                            degrees = 90
                        elif command == 'l':
                            degrees = -90
                        elif command == 'b':
                            degrees = 180
                        else:
                            # For custom rotation commands, ask user
                            try:
                                degrees = int(input("  Enter rotation degrees (90=right, -90=left, 180=back): "))
                            except ValueError:
                                degrees = 0
                                print("  Invalid degrees, assuming no rotation")

                        for i in range(len(all_candidates)):
                            all_candidates[i] = self.update_candidates_after_turn(
                                all_candidates[i], degrees
                            )
                    elif command == 'c':
                        # Continue straight - no position change
                        pass
                    else:
                        # Custom command - ask user how it affects position
                        print("  Custom command executed. Please specify how this affects position:")
                        try:
                            move_type = input("  Movement type ('f'=forward, 'r'=rotate, 'c'=continue): ").strip().lower()
                            if move_type == 'f':
                                for i in range(len(all_candidates)):
                                    all_candidates[i] = self.update_candidates_after_move(all_candidates[i])
                            elif move_type == 'r':
                                degrees = int(input("    Rotation degrees: "))
                                for i in range(len(all_candidates)):
                                    all_candidates[i] = self.update_candidates_after_turn(all_candidates[i], degrees)
                            # 'c' does nothing
                        except ValueError:
                            print("  Invalid input, skipping candidate updates")

                    # Show current status
                    if len(all_candidates) > 0:
                        current_common = self.intersect_candidate_history(all_candidates)
                        print(f"  Candidates remaining: {len(current_common)}")
                        if len(current_common) <= 10:
                            print("  Current candidates:")
                            for i, (x, y, theta) in enumerate(current_common):
                                print(f"    {i+1}: Position ({x},{y}), Facing {theta}°")

                    break
                else:
                    print("  Please enter a command")
        
        # Final analysis
        print("\n" + "-"*60)
        print("Final Analysis")
        print("-"*60)
        
        if not all_candidates:
            print("ERROR: No candidates found")
            return None
        
        # Find final common candidates
        common = self.intersect_candidate_history(all_candidates)
        
        print(f"Final consistent positions: {len(common)}")
        
        if not common:
            print("ERROR: Localization failed - no consistent position")
            return None
        
        if len(common) > 1:
            print(f"WARNING: Multiple candidates remain after {len(all_candidates)} samples: {common}")
            print("Selecting first candidate")
        
        x, y, orientation = list(common)[0]
        
        print("\n" + "="*60)
        print("Localization Complete!")
        print(f"  Position: (col={x}, row={y})")
        print(f"  Orientation: {orientation}° ({self.directions[orientation]})")
        print(f"  Samples taken: {len(all_candidates)}")
        print("="*60)
        
        return x, y, orientation
    
    def intersect_candidate_history(self, all_candidates):
        """
        Find candidates that are consistent across the entire movement history.
        
        Args:
            all_candidates: List of candidate lists from each sample
        
        Returns:
            set: Candidates that appear in all samples
        """
        if not all_candidates:
            return set()
        
        common = set(all_candidates[0])
        
        for i in range(1, len(all_candidates)):
            # Simply intersect - candidates have already been updated for movements/turns
            common = common.intersection(set(all_candidates[i]))
            
            if not common:
                # Lost tracking - use last sample
                return set(all_candidates[-1])
        
        return common
    
    def relocalize(self):
        """
        Quick relocalization using a single sensor reading.
        Useful after small movements where position is approximately known.
        
        Returns:
            tuple: (x, y, orientation) or None if failed
        """
        print("Relocalizing...")
        
        relative_walls = self.read_wall_sensors()
        if not relative_walls:
            return None
        
        # Try all orientations
        all_matches = []
        for orientation in [0, 90, 180, 270]:
            absolute_walls = self.get_absolute_walls(relative_walls, orientation)
            matches = self.match_wall_pattern(absolute_walls)
            all_matches.extend(matches)
        
        if not all_matches:
            print("  No matching position found")
            return None
        
        if len(all_matches) > 1:
            print(f"  Warning: {len(all_matches)} possible positions")
        
        x, y, orientation = all_matches[0]
        print(f"  Position: (col={x}, row={y}), Orientation: {orientation}°")
        
        return x, y, orientation


# Integration function for use with main script
def perform_initial_localization(maze, rows, cols, transmit_fn=None, receive_fn=None, packetize_fn=None):
    """
    Perform initial localization when rover is placed in maze.
    Uses adaptive sampling until a single configuration is found (max 50 samples).
    Args:
        maze: 2D maze array
        rows: Number of rows
        cols: Number of columns
        transmit_fn: Function to transmit data (optional)
        receive_fn: Function to receive data (optional)
        packetize_fn: Function to packetize data (optional)
    Returns:
        tuple: (x, y, orientation, localizer) or (None, None, None, None) if failed
    """
    localizer = LocalizationSystem(maze, rows, cols)
    # Inject communication functions if provided
    if transmit_fn:
        globals()['transmit'] = transmit_fn
    if receive_fn:
        globals()['receive'] = receive_fn
    if packetize_fn:
        globals()['packetize'] = packetize_fn
    result = localizer.localize_with_movement(max_samples=50)
    if result:
        x, y, orientation = result
        return x, y, orientation, localizer
    else:
        return None, None, None, None


# Example usage
if __name__ == "__main__":
    # This would be called from your main script
    print("Localization module loaded")
    print("Use: x, y, orientation, localizer = perform_initial_localization(MAZE, ROWS, COLS)")