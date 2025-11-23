from astar import astar, visualize_maze, path_to_commands, MAZE_TO_DROPOFF, MAZE_TO_LOADING
from comms import transmit, receive, packetize, SOURCE
import time
from execute_cmd import execute_cmds_with_safety, boot_and_align
from localization_interface import run_manual_localization

ROWS = len(MAZE_TO_LOADING)
COLS = len(MAZE_TO_LOADING[0])

if __name__ == "__main__":
    # Make sure we are connected to SOURCE
    print(f"Source: {SOURCE}")
    
    # ----------------------------------------------------------
    # ALIGN TO WALL AND CENTER ON BOOT
    # ----------------------------------------------------------
    boot_and_align()    
    print("\nWall alignment and centering complete.\n")
    
    # ----------------------------------------------------------
    # LOCALIZE ROBOT
    # ----------------------------------------------------------
    loc_x, loc_y, loc_orientation = run_manual_localization(
        MAZE_TO_DROPOFF, ROWS, COLS,
        transmit, receive, packetize,
        wall_thresh=6.0,
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
    start_orientation = loc_orientation

    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal (Loading Zone): (col={goal_cell[0]}, row={goal_cell[1]})")

    print("\nSearching for path with A*...")
    path = astar(start_cell, goal_cell, maze=MAZE_TO_LOADING)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")

        visualize_maze(path)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(cmds)
        
        execute_cmds_with_safety(cmds)
        print("\n✓ Reached loading zone!")
    else:
        print("No path found to loading zone!")
        exit()
    
    # ----------------------------------------------------------
    # WAIT FOR BLOCK PICKUP COMMAND
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("LOADING ZONE - Waiting for next action")
    print("="*60)
    print("Commands:")
    print("  'block'   - Run block pickup sequence")
    print("  'dropoff' - Skip pickup and go to dropoff")
    
    while True:
        cmd = input("\n> ").strip().lower()
        
        if cmd == 'block':
            print("\n" + "="*60)
            print("BLOCK PICKUP SEQUENCE")
            print("="*60)
            
            # ----------------------------------------------------------
            # BLOCK PICKUP SEQUENCE
            # TODO: Add your pickup commands here
            # ----------------------------------------------------------
            print("Block pickup sequence placeholder...")
            print("(Add pickup commands when ready)")
            
            # After pickup, robot should be at some position
            # Update these based on where pickup sequence ends
            pickup_end_cell = (0, 0)
            pickup_end_orientation = 0  # Update based on actual ending orientation
            
            print(f"\nBlock pickup complete!")
            print(f"Position: (col={pickup_end_cell[0]}, row={pickup_end_cell[1]}), facing {pickup_end_orientation}°")
            break
            
        elif cmd == 'dropoff':
            print("\nSkipping block pickup, proceeding to dropoff...")
            # Assume still at loading zone goal
            pickup_end_cell = goal_cell
            pickup_end_orientation = 0  # Adjust as needed
            break
            
        else:
            print(f"Unknown command: '{cmd}'")
            print("Type 'block' or 'dropoff'")
    
    # ----------------------------------------------------------
    # WAIT FOR DROPOFF NAVIGATION COMMAND
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("Ready for dropoff navigation")
    print("="*60)
    print("Type 'run dropoff' to navigate to dropoff zone")
    
    while True:
        cmd = input("\n> ").strip().lower()
        
        if cmd == 'run dropoff':
            print("\nStarting navigation to dropoff zone...")
            break
        else:
            print(f"Unknown command: '{cmd}'")
            print("Type 'run dropoff' to continue")
    
    # ----------------------------------------------------------
    # PATH PLANNING TO DROPOFF ZONE
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("NAVIGATION TO DROPOFF ZONE")
    print("="*60)
    
    start_cell = pickup_end_cell
    goal_cell = (0, 2)  ## CHANGE TO GIVEN DROP OFF ZONE
    start_orientation = pickup_end_orientation

    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal (Dropoff): (col={goal_cell[0]}, row={goal_cell[1]})")

    print("\nSearching for path with A*...")
    path = astar(start_cell, goal_cell, maze=MAZE_TO_DROPOFF)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")

        visualize_maze(path)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"Commands: {cmds}")
        
        if cmds:
            # Execute navigation commands (no 'bd' removal for now)
            execute_cmds_with_safety(cmds)
            print("\n✓ Navigation to dropoff zone complete!")
        else:
            print("Already at dropoff location - no movement needed")
        
        # TODO: Add block drop command here when implemented
        # Example: execute_block_drop() or execute_cmds_with_safety(['bd'])
        print("\n⚠ Block drop not implemented yet")
        print("   (Add block drop functionality here)")
        
    else:
        print("No path found to dropoff!")
        exit()
    
    print("\n" + "="*60)
    print("MISSION COMPLETE!")
    print("="*60)