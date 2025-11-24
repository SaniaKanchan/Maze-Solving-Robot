from astar import astar, visualize_maze, path_to_commands, MAZE_TO_DROPOFF, MAZE_TO_LOADING
from comms import transmit, receive, packetize, SOURCE
from block_pickup import block_scan_pickup, go_to_top_wall, rotate, read_u0, drive_forward
import time
from avoidance import wallalign
from execute_cmd import execute_cmds_with_safety, boot_and_align
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
    print("\n" + "="*60)
    print("INITIAL ALIGNMENT")
    print("="*60)
    boot_and_align()  
    wallalign()
    print("✓ Wall alignment and centering complete")
    
    # ----------------------------------------------------------
    # AUTOMATED LOCALIZATION
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("AUTOMATED LOCALIZATION")
    print("="*60)
    
    localizer = create_manual_localizer(
        MAZE_TO_DROPOFF, ROWS, COLS,
        transmit, receive, packetize,
        wall_thresh=6.0,
        min_dist=0.5
    )
    
    result = auto_localize(localizer, transmit, receive, packetize)
    
    if result and isinstance(result, tuple):
        loc_x, loc_y, loc_orientation = result
        print(f"✓ Localization complete: (col={loc_x}, row={loc_y}), facing {loc_orientation}°")
    else:
        print("✗ Automated localization failed. Exiting.")
        exit()
    
    # ----------------------------------------------------------
    # PATH PLANNING TO LOADING ZONE
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("NAVIGATION TO LOADING ZONE")
    print("="*60)
    
    start_cell = (loc_x, loc_y)
    goal_cell = (0, 2)
    start_orientation = loc_orientation

    print(f"Start: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal: (col={goal_cell[0]}, row={goal_cell[1]})")

    path = astar(start_cell, goal_cell, maze=MAZE_TO_LOADING)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        visualize_maze(path, maze=MAZE_TO_LOADING)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"Commands: {cmds}")
        
        execute_to_loading_zone(cmds, start_cell, goal_cell, path, MAZE_TO_LOADING)
        print("✓ Reached loading zone")
    else:
        print("✗ No path found to loading zone!")
        exit()
    
    # ----------------------------------------------------------
    # BLOCK PICKUP SEQUENCE
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("BLOCK PICKUP SEQUENCE")
    print("="*60)
    
    block_scan_pickup()
    boot_and_align()
    
    # Position after pickup
    pickup_end_cell = (0, 0)
    pickup_end_orientation = 0
    
    print(f"✓ Block pickup complete!")
    print(f"Position: (col={pickup_end_cell[0]}, row={pickup_end_cell[1]}), facing {pickup_end_orientation}°")
    
    # ----------------------------------------------------------
    # PATH PLANNING TO DROPOFF ZONE
    # ----------------------------------------------------------
    print("\n" + "="*60)
    print("NAVIGATION TO DROPOFF ZONE")
    print("="*60)
    
    start_cell = pickup_end_cell
    dropoff_cell = (5, 0)  # CHANGE THIS TO YOUR DROPOFF LOCATION
    start_orientation = pickup_end_orientation

    print(f"Start: (col={start_cell[0]}, row={start_cell[1]}) facing {start_orientation}°")
    print(f"Goal: (col={dropoff_cell[0]}, row={dropoff_cell[1]})")

    path = astar(start_cell, dropoff_cell, maze=MAZE_TO_DROPOFF)

    if path:
        print(f"✓ Path found! Length: {len(path)} cells")
        visualize_maze(path, maze=MAZE_TO_DROPOFF)
        cmds = path_to_commands(path, start_angle=start_orientation)
        print(f"Commands: {cmds}")
        
        if cmds:
            execute_to_dropoff_zone(cmds, start_cell, dropoff_cell, path, MAZE_TO_DROPOFF)
            print("✓ Reached dropoff zone")
        else:
            print("Already at dropoff location - no movement needed")
        
        # ----------------------------------------------------------
        # BLOCK DROP SEQUENCE
        # ----------------------------------------------------------
        print("\n" + "="*60)
        print("BLOCK DROP SEQUENCE")
        print("="*60)
        
        execute_cmds_with_safety(['bd'])
        print("✓ Block dropped")
        
    else:
        print("✗ No path found to dropoff!")
        exit()
    
    print("\n" + "="*60)
    print("✅ MISSION COMPLETE!")
    print("="*60)