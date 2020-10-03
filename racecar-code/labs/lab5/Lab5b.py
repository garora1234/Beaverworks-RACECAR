"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 5B - LIDAR Wall Following
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
FRONT_WINDOW = (-10, 10)
REAR_WINDOW = (170, 190)
RIGHT_WINDOW = (80, 100)
LEFT_WINDOW = (260, 280)
DESIRED_DISTANCE = 35
TURN_DISTANCE = 100
ACCEL_DISTANCE = 110

speed = 0
angle = 0

class State(IntEnum):
    accelerate = 0
    turn = 1
cur_state: State = State.accelerate
########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global cur_state
    global speed
    global angle

    cur_state = State.accelerate
    speed = 0
    angle = 0
    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message
    print(">> Lab 5B - LIDAR Wall Following")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global cur_state
    global speed
    global angle

    # TODO: Follow the wall to the right of the car without hitting anything.
    scan = rc.lidar.get_samples()
    _, forward_dist = rc_utils.get_lidar_closest_point(scan, FRONT_WINDOW)
    _, back_dist = rc_utils.get_lidar_closest_point(scan, REAR_WINDOW)
    _, right_dist = rc_utils.get_lidar_closest_point(scan, RIGHT_WINDOW)
    _, left_dist = rc_utils.get_lidar_closest_point(scan, LEFT_WINDOW)
    angle = rc_utils.remap_range(left_dist, 0, DESIRED_DISTANCE * 2, 1, -1)
    angle = rc_utils.clamp(angle, -1, 1)
    speed = 1
    if cur_state == State.accelerate:
        print("accelerating")
        if forward_dist <= TURN_DISTANCE:
            cur_state = State.turn
    if cur_state == State.turn:
        top_left = rc_utils.get_lidar_average_distance(scan, 315, 10)
        top_right = rc_utils.get_lidar_average_distance(scan, 45, 10)
        if top_left > top_right:
            angle = -1
        else:
            angle = 1
        print("turning")
        if forward_dist > ACCEL_DISTANCE:
            cur_state = State.accelerate
    print("Left: " + str(left_dist))
    print("Right: " + str(right_dist))

    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
