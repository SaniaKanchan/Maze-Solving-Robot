"""
Localization module for maze navigation using wall sensors.

This implements FEATURE-BASED LOCALIZATION (Wall Pattern Matching).

Method:
1. Read ultrasonic sensors to detect walls in all 4 cardinal directions
2. Generate a "wall signature" (binary pattern of which sides have walls)
3. Compare this signature against all known locations in the maze
4. Use movement and multiple readings to disambiguate between candidate positions
5. Converge to the true position and orientation
"""

import time


class LocalizationSystem:
    """Localizes the rover in a known maze using wall sensor readings."""
    
    def __init__(self, maze, rows, cols, transmit_fn, receive_fn, packetize_fn, 
                 wall_thresh=6.0, min_dist=0.5):
        """
        Initialize localization system.
        
        Args:
            maze: 2D list representing the maze (1=obstacle, 0=free)
            rows: Number of rows in maze
            cols: Number of columns in maze
            transmit_fn: Function to transmit data
            receive_fn: Function to receive data
            packetize_fn: Function to packetize commands
            wall_thresh: Distance threshold for wall detection (inches)
            min_dist: Minimum valid distance reading (inches)
        """
        self.maze = maze
        self.rows = rows
        self.cols = cols
        self.transmit = transmit_fn
        self.receive = receive_fn
        self.packetize = packetize_fn
        self.wall_thresh = wall_thresh
        self.min_dist = min_dist
        
        # Direction mappings
        self.directions = {
            0: 'right (East)',
            90: 'down (South)',
            180: 'left (West)',
            270: 'up (North)'
        }
        
        # Delta movements for each direction (dx, dy)
        self.deltas = {
            0: (1, 0),      # right: +x
            90: (0, 1),     # down: +y
            180: (-1, 0),   # left: -x
            270: (0, -1)    # up: -y
        }
        
        # Movement history tracking for manual localization
        self.candidate_history = []  # List of candidate sets from each reading
        self.movement_history = []   # List of movements: ('move', inches) or ('turn', degrees)
        self.cell_size = 12  # inches per cell
    
    def set_threshold(self, wall_thresh=None, min_dist=None):
        """Update threshold values."""
        if wall_thresh is not None:
            self.wall_thresh = wall_thresh
        if min_dist is not None:
            self.min_dist = min_dist
        print(f"Thresholds: WALL={self.wall_thresh}in, MIN={self.min_dist}in")
    
    def read_sensors(self):
        """
        Read all 6 ultrasonic sensors and return relative wall dict.
        
        Sensor layout:
            u0: Front middle
            u1: Left front
            u2: Left back
            u3: Back middle
            u4: Right back
            u5: Right front
        
        Returns:
            dict: {'front': bool, 'right': bool, 'back': bool, 'left': bool}
                  True = wall detected, False = no wall
        """
        packet = self.packetize("u0,u1,u2,u3,u4,u5")
        if not packet:
            print("ERROR: Failed to packetize sensor command")
            return None
        
        self.transmit(packet)
        time.sleep(0.15)
        resp, _ = self.receive()
        
        if not resp or len(resp) < 6:
            print(f"ERROR: Expected 6 responses, got {len(resp) if resp else 0}")
            return None
        
        try:
            u0 = float(resp[0][1])  # front
            u1 = float(resp[1][1])  # left front
            u2 = float(resp[2][1])  # left back
            u3 = float(resp[3][1])  # back
            u4 = float(resp[4][1])  # right back
            u5 = float(resp[5][1])  # right front
            
            # Use back sensors as primary, front as fallback
            # Left side: prefer left back (u2), fallback to left front (u1)
            left_dist = u2
            if not (self.min_dist < left_dist < 20):  # Bad reading (too close or too far)
                left_dist = u1
                print(f"    Using left front sensor (back sensor out of range)")
            
            # Right side: prefer right back (u4), fallback to right front (u5)
            right_dist = u4
            if not (self.min_dist < right_dist < 20):  # Bad reading
                right_dist = u5
                print(f"    Using right front sensor (back sensor out of range)")
            
            walls = {
                'front': self.min_dist < u0 < self.wall_thresh,
                'right': self.min_dist < right_dist < self.wall_thresh,
                'back': self.min_dist < u3 < self.wall_thresh,
                'left': self.min_dist < left_dist < self.wall_thresh
            }
            
            print(f"  Distances: F={u0:.1f} R=({u4:.1f},{u5:.1f}) B={u3:.1f} L=({u1:.1f},{u2:.1f})")
            print(f"  Using: L={left_dist:.1f} R={right_dist:.1f}")
            print(f"  Threshold: {self.wall_thresh}in | Min: {self.min_dist}in")
            wall_list = [d for d, v in walls.items() if v]
            print(f"  Walls detected: {wall_list if wall_list else 'none'}")
            return walls
            
        except (ValueError, IndexError, TypeError) as e:
            print(f"ERROR parsing sensor data: {e}")
            print(f"Response was: {resp}")
            return None
    
    def get_absolute_walls(self, rel_walls, orientation):
        """
        Convert relative wall readings to absolute maze directions.
        
        Args:
            rel_walls: dict with 'front', 'right', 'back', 'left'
            orientation: Current orientation (0, 90, 180, 270)
        
        Returns:
            dict: {'north': bool, 'east': bool, 'south': bool, 'west': bool}
        """
        rotation_map = {
            0:   {'front': 'east',  'right': 'south', 'back': 'west',  'left': 'north'},
            90:  {'front': 'south', 'right': 'west',  'back': 'north', 'left': 'east'},
            180: {'front': 'west',  'right': 'north', 'back': 'east',  'left': 'south'},
            270: {'front': 'north', 'right': 'east',  'back': 'south', 'left': 'west'}
        }
        mapping = rotation_map[orientation]
        return {mapping[k]: v for k, v in rel_walls.items()}
    
    def expected_walls(self, x, y):
        """
        Calculate expected walls at a given position based on maze structure.
        
        Args:
            x: Column index
            y: Row index
        
        Returns:
            dict: {'north': bool, 'east': bool, 'south': bool, 'west': bool}
        """
        walls = {'north': False, 'east': False, 'south': False, 'west': False}
        
        # Check maze boundaries
        if y == 0: walls['north'] = True
        if y == self.rows - 1: walls['south'] = True
        if x == 0: walls['west'] = True
        if x == self.cols - 1: walls['east'] = True
        
        # Check adjacent cells for obstacles
        if y > 0 and self.maze[y-1][x] == 1: walls['north'] = True
        if y < self.rows - 1 and self.maze[y+1][x] == 1: walls['south'] = True
        if x > 0 and self.maze[y][x-1] == 1: walls['west'] = True
        if x < self.cols - 1 and self.maze[y][x+1] == 1: walls['east'] = True
        
        return walls
    
    def find_matches(self, rel_walls):
        """
        Find all (x, y, orientation) that match the sensor reading.
        
        Args:
            rel_walls: Relative wall readings
        
        Returns:
            list: List of (x, y, orientation) tuples
        """
        candidates = []
        for ori in [0, 90, 180, 270]:
            abs_walls = self.get_absolute_walls(rel_walls, ori)
            for y in range(self.rows):
                for x in range(self.cols):
                    if self.maze[y][x] == 1:
                        continue
                    if self.expected_walls(x, y) == abs_walls:
                        candidates.append((x, y, ori))
        return candidates
    
    def can_move_forward(self, x, y, orientation):
        """Check if rover can move forward from given position."""
        dx, dy = self.deltas[orientation]
        nx, ny = x + dx, y + dy
        
        if nx < 0 or nx >= self.cols or ny < 0 or ny >= self.rows:
            return False
        if self.maze[ny][nx] == 1:
            return False
        return True
    
    def quick_localize(self):
        """
        Single reading localization.
        
        Returns:
            list: List of matching (x, y, orientation) tuples, or None
        """
        print("\n--- Quick Localization ---")
        walls = self.read_sensors()
        if not walls:
            return None
        
        matches = self.find_matches(walls)
        print(f"Found {len(matches)} possible positions:")
        for x, y, o in matches[:10]:
            print(f"  ({x}, {y}) facing {o}° {self.directions[o]}")
        if len(matches) > 10:
            print(f"  ... and {len(matches)-10} more")
        
        if len(matches) == 1:
            print(f"\n✓ Unique position found!")
        
        return matches
    
    def full_localize(self, max_samples=20):
        """
        Multi-sample localization with movement.
        
        Args:
            max_samples: Maximum samples before giving up
        
        Returns:
            tuple: (x, y, orientation) or None if failed
        """
        print("\n" + "="*50)
        print("FULL LOCALIZATION SEQUENCE")
        print("="*50)
        
        all_candidates = []
        
        for sample in range(max_samples):
            print(f"\n--- Sample {sample+1} ---")
            walls = self.read_sensors()
            if not walls:
                continue
            
            matches = self.find_matches(walls)
            print(f"This reading matches {len(matches)} positions")
            all_candidates.append(set(matches))
            
            # Intersect all candidates
            if len(all_candidates) > 1:
                common = all_candidates[0]
                for s in all_candidates[1:]:
                    common = common.intersection(s)
                print(f"Consistent across all samples: {len(common)}")
                
                if len(common) == 1:
                    x, y, o = list(common)[0]
                    print(f"\n{'='*50}")
                    print(f"✓ LOCALIZED: ({x}, {y}) facing {o}° {self.directions[o]}")
                    print(f"{'='*50}")
                    return (x, y, o)
                
                if len(common) == 0:
                    print("No consistent position - resetting")
                    all_candidates = [set(matches)]
            
            # Decide movement
            if sample < max_samples - 1:
                if walls['front']:
                    # Turn
                    if not walls['right']:
                        turn_dir, deg = 'right', 90
                    elif not walls['left']:
                        turn_dir, deg = 'left', -90
                    else:
                        turn_dir, deg = 'back', 180
                    
                    print(f"Wall ahead - turning {turn_dir}")
                    self.transmit(self.packetize(f"r0:{deg}"))
                    time.sleep(2)
                    self.receive()
                    
                    # Update candidates for turn
                    new_all = []
                    for cset in all_candidates:
                        new_all.append({(x, y, (o+deg)%360) for x,y,o in cset})
                    all_candidates = new_all
                else:
                    print("Moving forward 12 inches")
                    self.transmit(self.packetize("w0:12"))
                    time.sleep(3)
                    self.receive()
                    
                    # Update candidates for move
                    new_all = []
                    for cset in all_candidates:
                        new_set = set()
                        for x, y, o in cset:
                            dx, dy = self.deltas[o]
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.cols and 0 <= ny < self.rows:
                                if self.maze[ny][nx] == 0:
                                    new_set.add((nx, ny, o))
                        new_all.append(new_set)
                    all_candidates = new_all
        
        print("\nMax samples reached without convergence")
        return None


def create_localizer(maze, rows, cols, transmit_fn, receive_fn, packetize_fn,
                     wall_thresh=6.0, min_dist=0.5):
    """
    Factory function to create a LocalizationSystem.
    
    Args:
        maze: 2D maze array (0=free, 1=obstacle)
        rows: Number of rows
        cols: Number of columns
        transmit_fn: Transmit function
        receive_fn: Receive function
        packetize_fn: Packetize function
        wall_thresh: Wall detection threshold (inches)
        min_dist: Minimum valid distance (inches)
    
    Returns:
        LocalizationSystem instance
    """
    return LocalizationSystem(
        maze, rows, cols,
        transmit_fn, receive_fn, packetize_fn,
        wall_thresh, min_dist
    )


# ============== MANUAL TRACKING LOCALIZER ==============
class ManualTrackingLocalizer(LocalizationSystem):
    """
    Extended localizer that tracks manual movements for relocalization.
    
    Usage:
        1. User sends movement commands (w0:12, r0:90)
        2. Call record_move() or record_turn() after each command
        3. Call quick_localize() to get position using movement history
    """
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.candidate_history = []
        self.movement_history = []
        self.accumulated_distance = 0  # Track partial cell movements
    
    def reset_history(self):
        """Clear all tracking history."""
        self.candidate_history = []
        self.movement_history = []
        self.accumulated_distance = 0
        print("Movement history cleared")
    
    def record_move(self, inches):
        """
        Record a forward/backward movement. Call this after w0 command completes.
        Accumulates sub-cell movements until reaching 12 inches (1 cell).
        
        Args:
            inches: Distance moved (positive=forward, negative=backward)
        
        Examples:
            w0:3  + w0:3  + w0:6  → accumulates to 12", then updates 1 cell
            w0:12 → immediate 1 cell update
            w0:24 → immediate 2 cell update
        """
        # Add to accumulated distance
        self.accumulated_distance += inches
        
        # Calculate how many full cells we've moved
        cells_moved = int(self.accumulated_distance // self.cell_size)
        
        if cells_moved == 0:
            print(f"  Recorded: move {inches:+.0f}\" (accumulated: {self.accumulated_distance:.0f}\", waiting for 12\")")
            return
        
        # We've moved at least one full cell - process it
        is_backward = self.accumulated_distance < 0
        abs_cells = abs(cells_moved)
        
        # Store this cell movement in history (only once per full cell)
        self.movement_history.append(('move', cells_moved * self.cell_size))
        
        # Update remaining accumulation
        self.accumulated_distance = self.accumulated_distance % self.cell_size
        
        # Update all existing candidate sets for each cell moved
        for cell in range(abs_cells):
            new_history = []
            for cset in self.candidate_history:
                new_set = set()
                for x, y, o in cset:
                    dx, dy = self.deltas[o]
                    
                    # Reverse direction if moving backward
                    if is_backward:
                        dx, dy = -dx, -dy
                    
                    nx, ny = x + dx, y + dy
                    
                    # Check if new position is valid
                    if 0 <= nx < self.cols and 0 <= ny < self.rows:
                        if self.maze[ny][nx] == 0:
                            new_set.add((nx, ny, o))
                new_history.append(new_set)
            self.candidate_history = new_history
        
        direction = "backward" if is_backward else "forward"
        print(f"  Recorded: move {inches:+.0f}\" → {abs_cells} cell(s) {direction} (remaining: {self.accumulated_distance:.0f}\")")
    
    def record_turn(self, degrees):
        """
        Record a rotation. Call this after r0 command completes.
        Updates all candidate sets to reflect the turn.
        
        Args:
            degrees: Degrees turned (positive=clockwise, negative=counter-clockwise)
        """
        self.movement_history.append(('turn', degrees))
        
        # Update all existing candidate sets for this turn
        new_history = []
        for cset in self.candidate_history:
            new_set = {(x, y, (o + degrees) % 360) for x, y, o in cset}
            new_history.append(new_set)
        self.candidate_history = new_history
        
        print(f"  Recorded: turn {degrees}°")
    
    def quick_localize(self):
        """
        Take a sensor reading and combine with movement history to localize.
        
        Returns:
            tuple: (x, y, orientation) if unique, or list of candidates
        """
        print("\n--- Quick Localization (with history) ---")
        
        # Show movement tracking status
        cell_movements = len([m for m in self.movement_history if m[0] == 'move'])
        turns = len([m for m in self.movement_history if m[0] == 'turn'])
        print(f"Cell movements: {cell_movements} | Turns: {turns} | Accumulated: {self.accumulated_distance:.0f}\"")
        
        # Read current sensors
        walls = self.read_sensors()
        if not walls:
            return None
        
        # Find matches for current reading
        matches = self.find_matches(walls)
        print(f"Current reading matches {len(matches)} positions")
        
        # CRITICAL: Only update localization if we've completed a full cell movement
        if self.accumulated_distance > 0:
            # We're mid-cell - DON'T update candidates at all
            print(f"→ Mid-cell (accumulated: {self.accumulated_distance:.0f}\"/12\") - sensors shown but NOT updating localization")
            print(f"   Complete the cell movement (need {12 - self.accumulated_distance:.0f}\" more) before localization updates")
            
            # Show current candidates WITHOUT updating them
            if len(self.candidate_history) > 0:
                common = self.candidate_history[0]
                for cset in self.candidate_history[1:]:
                    common = common.intersection(cset)
                print(f"   Current candidates (unchanged): {len(common)} positions")
                if len(common) <= 10:
                    for x, y, o in common:
                        print(f"     ({x}, {y}) facing {o}° {self.directions[o]}")
            else:
                print(f"   No localization data yet - take a reading at cell boundary first")
            
            return list(common) if len(self.candidate_history) > 0 else None
        
        # We're at a cell boundary - safe to update localization
        self.candidate_history.append(set(matches))
        print(f"→ At cell boundary - added to history")
        
        # Intersect all candidate sets
        if len(self.candidate_history) == 1:
            common = self.candidate_history[0]
        else:
            common = self.candidate_history[0]
            for cset in self.candidate_history[1:]:
                common = common.intersection(cset)
        
        print(f"Consistent across {len(self.candidate_history)} readings: {len(common)} positions")
        
        # Display results
        if len(common) == 0:
            print("WARNING: No consistent position found!")
            print("  This might mean sensor error or tracking got out of sync")
            print("  Try 'reset' to clear history and start fresh")
            return None
        elif len(common) == 1:
            x, y, o = list(common)[0]
            print(f"\n{'='*50}")
            print(f"✓ LOCALIZED: ({x}, {y}) facing {o}° {self.directions[o]}")
            print(f"{'='*50}")
            return (x, y, o)
        else:
            print(f"Candidates remaining:")
            for x, y, o in list(common)[:10]:
                print(f"  ({x}, {y}) facing {o}° {self.directions[o]}")
            if len(common) > 10:
                print(f"  ... and {len(common)-10} more")
            return list(common)
    
    def get_status(self):
        """Print current tracking status."""
        print(f"\n--- Localization Status ---")
        print(f"Sensor readings taken: {len(self.candidate_history)}")
        print(f"Cell movements recorded: {len([m for m in self.movement_history if m[0] == 'move'])}")
        print(f"Turns recorded: {len([m for m in self.movement_history if m[0] == 'turn'])}")
        print(f"Accumulated distance: {self.accumulated_distance:.0f}\" (need {self.cell_size}\" for next cell)")
        
        if self.movement_history:
            print("\nMovement sequence:")
            total_cells = 0
            for i, (mtype, val) in enumerate(self.movement_history):
                if mtype == 'move':
                    cells = int(abs(val) // self.cell_size)
                    direction = "backward" if val < 0 else "forward"
                    total_cells += cells if val > 0 else -cells
                    print(f"  {i+1}. Move {val:+.0f}\" ({cells} cell(s) {direction})")
                else:
                    print(f"  {i+1}. Turn {val:+.0f}°")
            print(f"Net cells moved: {total_cells:+d}")
        
        if self.candidate_history:
            common = self.candidate_history[0]
            for cset in self.candidate_history[1:]:
                common = common.intersection(cset)
            print(f"Current candidates: {len(common)}")


def create_manual_localizer(maze, rows, cols, transmit_fn, receive_fn, packetize_fn,
                            wall_thresh=6.0, min_dist=0.5):
    """
    Factory function to create a ManualTrackingLocalizer.
    """
    return ManualTrackingLocalizer(
        maze, rows, cols,
        transmit_fn, receive_fn, packetize_fn,
        wall_thresh, min_dist
    )