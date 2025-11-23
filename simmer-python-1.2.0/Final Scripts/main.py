from astar import astar, visualize_maze, path_to_commands, MAZE_TO_DROPOFF, MAZE_TO_LOADING
from comms import transmit, receive, packetize,SOURCE
import time
from execute_cmd import execute_cmds_with_safety, boot_and_align
from localization_interface import run_manual_localization

ROWS = len(MAZE_TO_LOADING)
COLS = len(MAZE_TO_LOADING[0])

if __name__ == "__main__":
    #Make sure we are connected to SOURCE
    print(f"Source: {SOURCE}")
    # ----------------------------------------------------------
    # ALIGN TO WALL AND AND CENTER ON BOOT
    # ----------------------------------------------------------
    boot_and_align()    
    print("\nWall alignment and centering complete.\n")
    # ----------------------------------------------------------
    # Localize ROBOT with Random Movements
    # ----------------------------------------------------------
    #INSERT LOCALIZATION CODE HERE 
    loc_x, loc_y, loc_orientation = run_manual_localization(
        MAZE_TO_DROPOFF, ROWS, COLS,
        transmit, receive, packetize,
        wall_thresh=8.0,
        min_dist=0.5
    )
    
    if loc_x is None:
        print("Localization failed or cancelled. Exiting.")
        exit()
    # ----------------------------------------------------------
    # PATH PLANNING TO LOADING ZONE
    # ----------------------------------------------------------
    start_cell = (loc_x, loc_y)
    goal_cell = (0, 2)
    start_orientation = loc_orientation  # Degrees: 0=right, 90=up, 180=left, 270=down

    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal:  (col={goal_cell[0]}, row={goal_cell[1]})")

    print("\nSearching for path with A*...")
    path = astar(start_cell, goal_cell, maze=MAZE_TO_LOADING)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")

        visualize_maze(path)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(cmds)

    execute_cmds_with_safety(cmds)
    print("\nAll commands executed.")
    # ----------------------------------------------------------
    # BLOCK PICKUP SEQUENCE ONCE IN LOADING ZONE, ENDS ALIGNED TO TOP
    # ----------------------------------------------------------
    #
    # ----------------------------------------------------------
    # PATH PLANNING TO LOADING ZONE
    # ----------------------------------------------------------
    # start_cell = (0,0) 
    # goal_cell = (0, 2) ## CHANGE TO GIVEN DROP OFF ZONE
    # start_orientation = 0  # Degrees: 0=right, 90=up, 180=left, 270=down

    # print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    # print(f"Goal:  (col={goal_cell[0]}, row={goal_cell[1]})")

    # print("\nSearching for path with A*...")
    # path = astar(start_cell, goal_cell, maze=MAZE_TO_LOADING)

    # if path:
    #     print(f"✓ Path found! Length: {len(path)} cells")
    #     print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")

    #     visualize_maze(path)
    #     cmds = path_to_commands(path, start_angle=start_orientation)
    #     print(cmds)
    
    # #remove one last drive command from cmds to for dropoff, and add dropoff command
    # cmds = cmds[:-1]
    # cmds.append("bd")  # Replace "dropoff_command" with the actual command for dropoff
    # execute_cmds_with_safety(cmds)
    # print("\nAll commands executed.")