from astar import astar, visualize_maze, path_to_commands, MAZE_TO_DROPOFF, MAZE_TO_LOADING
from comms import transmit, receive, packetize, SOURCE
from block_pickup import block_scan_pickup, go_to_top_wall, rotate, read_u0, drive_forward
import time
from avoidance import wallalign
from execute_cmd import execute_cmds_with_safety, boot_and_align
from localization_interface import run_manual_localization
from automated_localization import auto_localize
from localization_submodule import create_manual_localizer
from execute_with_viz import execute_to_loading_zone, execute_to_dropoff_zone

ROWS = len(MAZE_TO_LOADING)
COLS = len(MAZE_TO_LOADING[0])

if __name__ == "__main__":
    # Make sure we are connected to SOURCE
    print(f"Source: {SOURCE}")
    
    # ----------------------------------------------------------
    # ALIGN TO WALL AND CENTER ON BOOT
    # ----------------------------------------------------------
    boot_and_align()  
    wallalign()
    
    print("\nWall alignment and centering complete.\n")
    
    # ----------------------------------------------------------
    # LOCALIZE ROBOT - Choose Method
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("LOCALIZATION METHOD")
    print("="*60)
    print("Choose localization method:")
    print("  'auto'   - Automated localization (robot moves itself)")
    print("  'manual' - Manual localization (you control movements)")
    
    while True:
        choice = input("\n> ").strip().lower()
        
        if choice == 'auto':
            print("\nStarting automated localization...")
            # Create localizer for auto mode
            localizer = create_manual_localizer(
                MAZE_TO_DROPOFF, ROWS, COLS,
                transmit, receive, packetize,
                wall_thresh=6.0,
                min_dist=0.5
            )
            
            result = auto_localize(localizer, transmit, receive, packetize)
            
            if result and isinstance(result, tuple):
                loc_x, loc_y, loc_orientation = result
                break
            else:
                print("\nAutomated localization failed. Try manual mode.")
                print("Type 'manual' to switch to manual localization")
                continue
                
        elif choice == 'manual':
            print("\nStarting manual localization...")
            loc_x, loc_y, loc_orientation = run_manual_localization(
                MAZE_TO_DROPOFF, ROWS, COLS,
                transmit, receive, packetize,
                wall_thresh=6.0,
                min_dist=0.5
            )
            
            if loc_x is not None:
                break
            else:
                print("Manual localization cancelled.")
                exit()
        else:
            print(f"Unknown choice: '{choice}'")
            print("Type 'auto' or 'manual'")
    
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

        visualize_maze(path, maze=MAZE_TO_LOADING)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"Commands: {cmds}")
        
        # Execute with live visualization
        execute_to_loading_zone(cmds, start_cell, goal_cell, path, MAZE_TO_LOADING)
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
            # ----------------------------------------------------------
            block_scan_pickup()
            #go_to_top_wall(rotate, read_u0, drive_forward)
            boot_and_align()
            print("Block pickup sequence complete!")
            
            # Use relocalized position
            pickup_end_cell = (0, 0)
            pickup_end_orientation = 0
            
            print(f"\nBlock pickup complete!")
            print(f"Position: (col={pickup_end_cell[0]}, row={pickup_end_cell[1]}), facing {pickup_end_orientation}°")
            
            break
            
        elif cmd == 'dropoff':
            print("\nSkipping block pickup, proceeding to dropoff...")
            # Assume still at loading zone goal
            pickup_end_cell = goal_cell
            pickup_end_orientation = 270  # Adjust as needed
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
    
    start_cell = pickup_end_cell  # Start from position after pickup
    dropoff_cell = (5, 1)  ## CHANGE TO GIVEN DROP OFF ZONE
    start_orientation = pickup_end_orientation  # Use orientation after pickup

    print(f"\nStart: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal (Dropoff): (col={dropoff_cell[0]}, row={dropoff_cell[1]})")

    print("\nSearching for path with A*...")
    path = astar(start_cell, dropoff_cell, maze=MAZE_TO_DROPOFF)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        print(f"  Path: {' → '.join([f'({x},{y})' for x, y in path])}")

        visualize_maze(path, maze=MAZE_TO_DROPOFF)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"Commands: {cmds}")
        
        if cmds:
            # Execute with live visualization
            execute_to_dropoff_zone(cmds, start_cell, dropoff_cell, path, MAZE_TO_DROPOFF)
        else:
            print("Already at dropoff location - no movement needed")
        
        # DROP BLOCK HERE!
        print("\n" + "="*60)
        print("🔽 BLOCK DROP SEQUENCE")
        print("="*60)
        
        #remove one last drive command from cmds to for dropoff, and add dropoff command
        #cmds = cmds[:-1]
        #cmds.append("bd")
        execute_cmds_with_safety(['bd'])
        
    else:
        print("No path found to dropoff!")
        exit()
    
    print("\n" + "="*60)
    print("MISSION COMPLETE!")
    print("="*60)