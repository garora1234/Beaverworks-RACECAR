"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3A - Depth Camera Safety Stop
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum
import statistics

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
class State(IntEnum):
    driving = 0
    stop = 1
cur_state: State = State.driving
speed = 0
dist = 0
distances = []
car_speed = 0
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    # Have the car begin at a stop
    rc.drive.stop()
    global speed
    global car_speed
    global dist
    global distances
    global cur_state
    cur_state = State.driving

    # Print start message
    print(
        ">> Lab 3A - Depth Camera Safety Stop\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Right bumper = override safety stop\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = print current speed and angle\n"
        "   B button = print the distance at the center of the depth image"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state
    global speed
    global car_speed
    global dist
    global distances

    depth_image = rc.camera.get_depth_image()
    center_row = rc.camera.get_height() // 2
    center_col = rc.camera.get_width() // 2
    coord = (center_row, center_col)
    average_distance = rc_utils.get_pixel_average_distance(depth_image, coord)
    distances.append(float(average_distance))
    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt
    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    if cur_state == State.driving:
        if len(distances) > 60:
            car_speed = (statistics.mean(distances) * 60) // 100
            dist = rc_utils.remap_range(car_speed, 0, 1, 1, 300)
        rc.drive.set_speed_angle(speed, angle)
        if average_distance <= dist and average_distance > 0:
            cur_state = State.stop
    
    if cur_state == State.stop:
        rc.drive.stop()
        distances.clear()
    # Allow the user to override safety stop by holding the right bumper.
        if rc.controller.is_down(rc.controller.Button.RB):
            cur_state = State.driving
    
    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the depth image center distance when the B button is held down
    if rc.controller.is_down(rc.controller.Button.B):
        print("Average distance:", average_distance)

    # Display the current depth image
    rc.display.show_depth_image(depth_image)

    # TODO (stretch goal): Prevent forward movement if the car is about to drive off a
    # ledge.  ONLY TEST THIS IN THE SIMULATION, DO NOT TEST THIS WITH A REAL CAR.

    # TODO (stretch goal): Tune safety stop so that the car is still able to drive up
    # and down gentle ramps.
    # Hint: You may need to check distance at multiple points.


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
