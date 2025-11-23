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
        # Send command to read all 6 ultrasonic sensors
        cmd = "u0,u1,u2,u3,u4,u5"
        try:
            packet = globals()['packetize'](cmd)
            if not packet:
                print("ERROR: Failed to packetize sensor command")
                return None
            globals()['transmit'](packet)
            time.sleep(0.15)  # Slightly longer delay for 6 sensors
            responses, _ = globals()['receive']()
            WALL_THRESHOLD = 5  # inches - adjust based on your setup
            MIN_DISTANCE = 0.5  # Minimum valid distance reading
            if not responses or len(responses) < 6:
                print(f"ERROR: Expected 6 sensor responses, got {len(responses) if responses else 0}")
                return None
            # Extract distances from all sensors
            u0_dist = float(responses[0][1])  # front
            u1_dist = float(responses[1][1])  # left front
            u2_dist = float(responses[2][1])  # left back
            u3_dist = float(responses[3][1])  # back
            u4_dist = float(responses[4][1])  # right back
            u5_dist = float(responses[5][1])  # right front
            # Aggregate readings for each cardinal direction
            walls = {
                'front': MIN_DISTANCE < u0_dist < WALL_THRESHOLD,
                'right': MIN_DISTANCE < (u4_dist + u5_dist) / 2 < WALL_THRESHOLD,
                'back': MIN_DISTANCE < u3_dist < WALL_THRESHOLD,
                'left': MIN_DISTANCE < (u1_dist + u2_dist) / 2 < WALL_THRESHOLD
            }
            # Debug output
            print(f"    Sensor distances: F={u0_dist:.1f} R={u4_dist:.1f}/{u5_dist:.1f} B={u3_dist:.1f} L={u1_dist:.1f}/{u2_dist:.1f}")
            return walls
        except (ValueError, IndexError, TypeError) as e:
            print(f"ERROR parsing sensor data: {e}")
            return None
    
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
            
            # Read wall sensors
            print("  Reading wall sensors...")
            relative_walls = self.read_wall_sensors()
            
            if not relative_walls:
                print("  ERROR: Failed to read sensors")
                continue
            
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
                # Start with first sample's candidates
                common = set(all_candidates[0])
                
                # For subsequent samples, find candidates that are consistent
                for i in range(1, len(all_candidates)):
                    next_common = set()
                    
                    for x1, y1, o1 in common:
                        for x2, y2, o2 in all_candidates[i]:
                            # Check if o2 is reachable from o1 with forward movement
                            dx, dy = self.direction_deltas[o1]
                            # After moving ~3 inches in a 12-inch cell, might still be in same cell
                            if (x2, y2, o2) == (x1, y1, o1) or (x2, y2, o2) == (x1 + dx, y1 + dy, o1):
                                next_common.add((x2, y2, o2))
                    
                    common = next_common
                    
                    if not common:
                        print("WARNING: No consistent position found, using last sample")
                        common = set(all_candidates[-1])
                        break
                
                print(f"  Consistent positions remaining: {len(common)}")
                
                # Check if we have converged to a single configuration
                if len(common) == 1:
                    print("  Converged to single configuration!")
                    break
            
            # Move forward slightly for next reading (unless this is the last sample)
            if sample < max_samples - 1:
                print("  Moving forward 12 inches...")
                cmd = "w0:12"
                packet = globals()['packetize'](cmd)
                if packet:
                    globals()['transmit'](packet)
                    time.sleep(4)  # Allow time for movement
                    globals()['receive']()  # Wait for completion
        
        # Final analysis
        print("\n" + "-"*60)
        print("Final Analysis")
        print("-"*60)
        
        if not all_candidates:
            print("ERROR: No candidates found")
            return None
        
        # Find final common candidates
        common = set(all_candidates[0])
        for i in range(1, len(all_candidates)):
            next_common = set()
            for x1, y1, o1 in common:
                for x2, y2, o2 in all_candidates[i]:
                    dx, dy = self.direction_deltas[o1]
                    if (x2, y2, o2) == (x1, y1, o1) or (x2, y2, o2) == (x1 + dx, y1 + dy, o1):
                        next_common.add((x2, y2, o2))
            common = next_common
            if not common:
                common = set(all_candidates[-1])
                break
        
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