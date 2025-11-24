"""
Enhanced Command Execution with Live Visualization

Executes path commands while showing robot position in real-time.
"""

from execute_cmd import execute_cmds_with_safety
from astar import visualize_robot_position, CELL_SIZE_INCHES, MOVE_INCREMENT


def execute_with_live_visualization(cmds, start_pos, goal_pos, path, maze, phase_name="Navigation"):
    """
    Execute commands while showing robot position after each cell movement.
    
    Args:
        cmds: List of command strings (e.g., ['r0:90', 'w0:3', ...])
        start_pos: (x, y) starting position tuple
        goal_pos: (x, y) goal position tuple
        path: List of (x, y) path positions from A*
        maze: 2D maze array
        phase_name: Name of current phase (e.g., "To Loading Zone", "To Dropoff")
    """
    print("\n" + "="*60)
    print(f"EXECUTING PATH - {phase_name}")
    print("="*60)
    
    # Show initial position
    visualize_robot_position(start_pos, goal_pos, path, maze, 
                            f"Starting navigation to {goal_pos}")
    
    # Track current position and orientation
    current_pos = list(start_pos)  # [x, y]
    current_orientation = None  # Will be set from first rotation or inferred
    
    # Determine initial orientation from first movement in path
    if len(path) > 1:
        dx = path[1][0] - path[0][0]
        dy = path[1][1] - path[0][1]
        if dx == 1:
            current_orientation = 0  # East
        elif dx == -1:
            current_orientation = 180  # West
        elif dy == 1:
            current_orientation = 90  # South
        elif dy == -1:
            current_orientation = 270  # North
    
    # Track path progress
    path_index = 0  # Current position in path list
    steps_per_cell = CELL_SIZE_INCHES // MOVE_INCREMENT  # 12 / 3 = 4 steps per cell
    
    # Execute commands and track position
    i = 0
    steps_in_current_cell = 0
    
    while i < len(cmds):
        cmd = cmds[i]
        
        # Execute the command
        print(f"\n[Command {i+1}/{len(cmds)}] Executing: {cmd}")
        execute_cmds_with_safety([cmd])
        
        # Update position based on command
        if cmd.startswith('r0:'):
            # Rotation command - update orientation
            degrees = int(cmd.split(':')[1])
            if current_orientation is not None:
                current_orientation = (current_orientation + degrees) % 360
            print(f"  → Rotated to {current_orientation}°")
            
        elif cmd.startswith('w0:'):
            # Movement command - track steps
            inches = int(cmd.split(':')[1])
            steps_in_current_cell += 1
            
            # Check if we've completed a full cell (4 steps of 3 inches each)
            if steps_in_current_cell >= steps_per_cell:
                # Move to next cell in path
                path_index += 1
                if path_index < len(path):
                    current_pos = list(path[path_index])
                    
                    # Show updated position
                    cells_remaining = len(path) - path_index - 1
                    visualize_robot_position(
                        tuple(current_pos), goal_pos, path, maze,
                        f"Moved to cell ({current_pos[0]}, {current_pos[1]}) - {cells_remaining} cells to goal"
                    )
                
                steps_in_current_cell = 0
        
        i += 1
    
    # Reached goal
    print("\n" + "="*60)
    print(f"✓ REACHED GOAL: {goal_pos}")
    print("="*60)
    visualize_robot_position(goal_pos, goal_pos, path, maze, 
                            f"Arrived at destination!")


def execute_to_loading_zone(cmds, start_pos, goal_pos, path, maze):
    """Execute path to loading zone with visualization."""
    execute_with_live_visualization(cmds, start_pos, goal_pos, path, maze, 
                                    "Navigation to Loading Zone")
    
    print("\n" + "="*60)
    print("📦 REACHED LOADING ZONE")
    print("="*60)
    print("Ready for block pickup sequence...")


def execute_to_dropoff_zone(cmds, start_pos, goal_pos, path, maze):
    """Execute path to dropoff zone with visualization."""
    execute_with_live_visualization(cmds, start_pos, goal_pos, path, maze,
                                    "Navigation to Dropoff Zone")
    
    print("\n" + "="*60)
    print("📍 REACHED DROPOFF ZONE")
    print("="*60)
    print("Ready for block drop sequence...")


# Simple non-visualized execution (for backward compatibility)
def execute_path(cmds):
    """Execute commands without visualization (simple wrapper)."""
    execute_cmds_with_safety(cmds)