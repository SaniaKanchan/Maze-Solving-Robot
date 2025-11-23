
import time
from queue import PriorityQueue

# Maze configuration
ROWS = 5  # Number of rows in the maze
COLS = 5  # Number of columns in the maze
MAZE = [[0, 0, 0, 0, 0],  # 0 = free space, 1 = obstacle
        [0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0]]

# Constants for movement
CELL_SIZE_INCHES = 12  # Distance between cell centers
MOVE_INCREMENT = 6     # Distance per movement command

# Communication placeholders (will be set by calling code)
transmit = lambda *args: None
receive = lambda *args: None
packetize = lambda *args: None

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
def neighbors(node, maze, rows, cols):
    dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # left, up, right, down
    result = []
    for dx, dy in dirs:
        nx, ny = node.x + dx, node.y + dy
        # MAZE[row][col] = MAZE[y][x]
        if 0 <= nx < cols and 0 <= ny < rows and maze[ny][nx] == 0:
            result.append((nx, ny))
    return result

# --- A* search ---
def astar(start, goal, maze, rows, cols):
    """
    Find shortest path from start to goal using A*.
    
    Args:
        start: (x, y) tuple where x=column, y=row
        goal: (x, y) tuple where x=column, y=row
        maze: 2D list representing the maze
        rows: number of rows
        cols: number of columns
    
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
        
        for nx, ny in neighbors(current, maze, rows, cols):
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
def visualize_maze(path=None, maze=None, rows=None, cols=None):
    """Print the maze with optional path overlay."""
    if maze is None:
        maze = MAZE
    if rows is None:
        rows = ROWS
    if cols is None:
        cols = COLS
    print("\nMaze visualization:")
    print(f"  {rows}x{cols} grid (0=free, 1=obstacle)")
    
    path_set = set(path) if path else set()
    
    for y in range(rows):
        row_str = f"  Row {y}: "
        for x in range(cols):
            if path and (x, y) in path_set:
                if (x, y) == path[0]:
                    row_str += "S "  # Start
                elif (x, y) == path[-1]:
                    row_str += "G "  # Goal
                else:
                    row_str += "• "  # Path
            elif maze[y][x] == 1:
                row_str += "█ "  # Obstacle
            else:
                row_str += "· "  # Free
        print(row_str)

# --- Main execution ---
if __name__ == "__main__":
    print("="*60)
    print("A* Path Planning for SimMeR with Relocalization")
    print("="*60)
    
    # Define start and goal (x, y) where x=column, y=row
    start_cell = (4, 1)  # col 7, row 3
    goal_cell = (0, 2)   # col 0, row 2
    start_orientation = 270  # facing up initially
    
    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal:  (col={goal_cell[0]}, row={goal_cell[1]})")
    print(f"Note: Cell centers are 1y2 inches apart")
    
    # Find path using A*
    print("\nSearching for path with A*...")
    path = astar(start_cell, goal_cell, MAZE, ROWS, COLS)
    
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