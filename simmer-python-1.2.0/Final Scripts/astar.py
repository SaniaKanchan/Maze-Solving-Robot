from comms import transmit, receive, packetize, SOURCE
import time 
import math
from queue import PriorityQueue

# --- Maze definitions (4x8 grid, 1=blocked, 0=free) ---
# row 0 = top, col 0 = left

# MAZE 1: For traveling TO loading zone - top-left 4 cells blocked
MAZE_TO_LOADING = [
    [1, 1, 0, 0, 1, 0, 1, 0],  # row 0
    [1, 1, 1, 0, 0, 0, 0, 0],  # row 1
    [0, 1, 0, 1, 1, 0, 1, 0],  # row 2
    [0, 0, 0, 0, 0, 0, 1, 0]   # row 3
]

# MAZE 2: For traveling FROM loading zone to drop-off - top-left 4 cells open
MAZE_TO_DROPOFF = [
    [0, 0, 0, 0, 1, 0, 1, 0],  # row 0 - top-left now open
    [0, 0, 1, 0, 0, 0, 0, 0],  # row 1 - top-left now open
    [0, 1, 0, 1, 1, 0, 1, 0],  # row 2
    [0, 0, 0, 0, 0, 0, 1, 0]   # row 3
]

ROWS = len(MAZE_TO_LOADING)
COLS = len(MAZE_TO_LOADING[0])

# Cell size and movement parameters (all in inches)
CELL_SIZE_INCHES = 12  # Each maze cell is 12 inches (center to center)
MOVE_INCREMENT = 3     # Move in 3-inch increments

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
def neighbors(node, maze):
    dirs = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # left, up, right, down
    result = []
    for dx, dy in dirs:
        nx, ny = node.x + dx, node.y + dy
        # maze[row][col] = maze[y][x]
        if 0 <= nx < COLS and 0 <= ny < ROWS and maze[ny][nx] == 0:
            result.append((nx, ny))
    return result

# --- A* search ---
def astar(start, goal, maze=MAZE_TO_LOADING):
    """
    Find shortest path from start to goal using A*.
    
    Args:
        start: (x, y) tuple where x=column, y=row
        goal: (x, y) tuple where x=column, y=row
        maze: 2D list representing the maze (default: MAZE_TO_LOADING)
    
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
        
        for nx, ny in neighbors(current, maze):
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

# --- Visualize maze with path ---
def visualize_maze(path=None, maze=MAZE_TO_LOADING):
    """Print the maze with optional path overlay."""
    print("\nMaze visualization:")
    print(f"  {ROWS}x{COLS} grid (0=free, 1=obstacle)")
    
    path_set = set(path) if path else set()
    
    for y in range(ROWS):
        row_str = f"  Row {y}: "
        for x in range(COLS):
            if (x, y) in path_set:
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
    print(f"Source: {SOURCE}")
    print("="*60)
    
    # EXAMPLE 1: Path TO loading zone (top-left blocked)
    print("\n" + "="*60)
    print("SCENARIO 1: Traveling TO loading zone")
    print("="*60)
    
    start_cell = (7, 3)  # Starting position
    loading_zone = (0, 2)   # Loading zone location
    start_orientation = 270  # facing up initially
    
    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal:  (col={loading_zone[0]}, row={loading_zone[1]}) [LOADING ZONE]")
    
    path_to_loading = astar(start_cell, loading_zone, maze=MAZE_TO_LOADING)
    
    if path_to_loading:
        print(f"✓ Path found! Length: {len(path_to_loading)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path_to_loading])}")
        visualize_maze(path_to_loading, maze=MAZE_TO_LOADING)
        cmds_to_loading = path_to_commands(path_to_loading, start_angle=start_orientation)
        print(f"\nGenerated {len(cmds_to_loading)} commands to reach loading zone")
    
    # EXAMPLE 2: Path FROM loading zone to drop-off (top-left open)
    print("\n" + "="*60)
    print("SCENARIO 2: Traveling FROM loading zone to drop-off")
    print("="*60)
    
    dropoff_zone = (7, 0)  # Drop-off zone location
    loading_orientation = 180  # Assume facing left after loading
    
    print(f"\nStart: (col={loading_zone[0]}, row={loading_zone[1]}) [LOADING ZONE] facing {loading_orientation}°")
    print(f"Goal:  (col={dropoff_zone[0]}, row={dropoff_zone[1]}) [DROP-OFF ZONE]")
    
    path_from_loading = astar(loading_zone, dropoff_zone, maze=MAZE_FROM_LOADING)
    
    if path_from_loading:
        print(f"✓ Path found! Length: {len(path_from_loading)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path_from_loading])}")
        visualize_maze(path_from_loading, maze=MAZE_FROM_LOADING)
        cmds_from_loading = path_to_commands(path_from_loading, start_angle=loading_orientation)
        print(f"\nGenerated {len(cmds_from_loading)} commands from loading zone to drop-off")