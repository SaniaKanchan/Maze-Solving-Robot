'''
This file stores the configuration information for the simulator.
'''
# This file is part of SimMeR, an educational mechatronics robotics simulator.
# Initial development funded by the University of Toronto MIE Department.
# Copyright (C) 2023  Ian G. Bennett
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import math
import pygame.math as pm
from devices.motors import MotorSimple
from devices.ultrasonic import Ultrasonic
from devices.gyroscope import Gyroscope
from devices.compass import Compass
from devices.infrared import Infrared
from devices.drive import Drive

# Control Flags and Setup
rand_error = False          # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = False           # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [0.05, 1]   # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = '127.0.0.1'
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = 'ascii'
frame_start = '['
frame_end = ']'

# General communication settings
round_digits = 3

# Block information
block_position = [16, 15]        # Block starting location
block_rotation = 0              # Block rotation (deg)
block_size = 2                  # Block side length in inches

# Robot information
robot_start_position = [54, 18]  # Robot starting location (in)
robot_start_rotation = 0         # Robot starting rotation (deg)

# Robot dimensions: 190mm x 166mm rectangle
robot_width = 6.535   # 166mm in inches (short side, left-right)
robot_height = 7.480  # 190mm in inches (long side, front-back)

# Robot outline as a rectangle
robot_outline = [            
    pm.Vector2(-robot_width/2, robot_height/2),   # Front-left
    pm.Vector2(robot_width/2, robot_height/2),    # Front-right
    pm.Vector2(robot_width/2, -robot_height/2),   # Rear-right
    pm.Vector2(-robot_width/2, -robot_height/2)   # Rear-left
]

# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length


# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 8                     # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format

block_thickness = 0.25      # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0) # Tuple with robot perimeter color in (R,G,B) format



### DEVICE CONFIGURATION ###
# Motors (Two wheels, symmetrical and centered on long sides)
# Wheels placed at the outer edges of the rectangle

m0_info = {
    'id': 'm0',
    'position': [robot_width/2, 0],  # Right wheel (centered on long side)
    'rotation': 0,
    'visible': True,
}

m1_info = {
    'id': 'm1',
    'position': [-robot_width/2, 0],  # Left wheel (centered on long side)
    'rotation': 0,
    'visible': True,
}

motors = {
    'm0': MotorSimple(m0_info),
    'm1': MotorSimple(m1_info),
}

# Drives

# Forward motion
w0_info = {
    'id': 'w0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 4],       # Forward 4 in/s
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
}

# Rotational motion
r0_info = {
    'id': 'r0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 0],        # No linear motion
    'ang_velocity': 60,        # 60 deg/s
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, -1],  # Opposite directions for rotation
}

# Add drives to dictionary
drives = {
    'w0': Drive(w0_info),
    'r0': Drive(r0_info)
}

# Sensors
# Sensor dimensions
long_side = 1.9
short_side = 0.5
outline = [
    pm.Vector2(-long_side/2, -short_side/2),
    pm.Vector2(-long_side/2, short_side/2),
    pm.Vector2(long_side/2, short_side/2),
    pm.Vector2(long_side/2, -short_side/2)
]

# Calculate sensor positions
# Front sensor: 68.5mm from front = 2.697 inches from front
# Other sensors: 11.5mm from edge = 0.453 inches from edge
# Side sensors: 20mm apart = 0.787 inches apart

front_sensor_offset = robot_height/2 - 2.697  # Distance from center to front sensor
side_edge_offset = robot_width/2 - 0.453       # Distance from center to side sensors (horizontal)
side_sensor_spacing = 4                    # Vertical spacing between side sensors

u0_info = {  # Front middle
    'id': 'u0',
    'position': [0, front_sensor_offset],
    'height': 6,
    'rotation': 0,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

ub_info = {  # BLOCK PICKUP SENSOR
    'id': 'u0',
    'position': [0, front_sensor_offset-1],
    'height': 1,
    'rotation': 0,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

u1_info = {  # Left front
    'id': 'u1',
    'position': [side_edge_offset, side_sensor_spacing/2],
    'height': 6,
    'rotation': -90,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}
u2_info = {  # Left back
    'id': 'u2',
    'position': [side_edge_offset, -side_sensor_spacing/2],
    'height': 6,
    'rotation': -90,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

u3_info = {  # Back middle
    'id': 'u3',
    'position': [0, -robot_height/2 + 0.453],
    'height': 6,
    'rotation': 180,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

u4_info = {  # Right back
    'id': 'u4',
    'position': [-side_edge_offset, -side_sensor_spacing/2],
    'height': 6,
    'rotation': 90,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

u5_info = {  # Right front
    'id': 'u5',
    'position': [-side_edge_offset, side_sensor_spacing/2],
    'height': 6,
    'rotation': 90,
    'error': 0.01,
    'outline': outline,
    'visible': True,
    'visible_measurement': True
}

sensors = {
    'u0': Ultrasonic(u0_info),
    'u1': Ultrasonic(u1_info),
    'u2': Ultrasonic(u2_info),
    'u3': Ultrasonic(u3_info),
    'u4': Ultrasonic(u4_info),
    'u5': Ultrasonic(u5_info),
    'ub': Ultrasonic(ub_info),
}