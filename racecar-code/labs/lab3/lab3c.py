"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3C - Depth Camera Wall Parking
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
DESIRED_DISTANCE = 20
BACK_DISTANCE = DESIRED_DISTANCE * 0.8
FORWARD_DISTANCE = DESIRED_DISTANCE * 2
speed: float = 0.0
angle: float = 0.0
variance: float = 0.3
POINT1 = int(rc.camera.get_width() * 0.3) 
POINT2 = int(rc.camera.get_width() * 0.7)

class State(IntEnum):
    align = 0
    stop = 1
    reverse = 2
########################################################################################
# Functions
########################################################################################

def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_state
    global variance
    cur_state = State.align
    # Have the car begin at a stop
    rc.drive.stop()
    speed = 0
    angle = 0

    # Print start message
    print(">> Lab 3C - Depth Camera Wall Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state
    global variance
    
    depth_image = rc.camera.get_depth_image()
    #closest_point = rc_utils.get_closest_pixel(depth_image)
    #distance = rc_utils.get_pixel_average_distance(depth_image, closest_point)
    point1 = [140, POINT1]
    point2 = [140, POINT2]
    pt1_dist = rc_utils.get_pixel_average_distance(depth_image, point1)
    pt2_dist = rc_utils.get_pixel_average_distance(depth_image, point2)
    pt_ang = pt1_dist - pt2_dist
    distance = min(pt1_dist, pt2_dist)
    #print(pt1_dist)
    #print(pt2_dist)
    rc.display.show_depth_image(depth_image, points = [point1, point2])
    # TODO: Park the car 20 cm away from the closest wall with the car directly facing
    # the wall
    angle = rc_utils.remap_range(pt_ang, -25, 25, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)
    print(angle)
    if cur_state == State.align:
        speed = rc_utils.remap_range(distance, 0, 700, 0, 1)
        speed = rc_utils.clamp(speed, 0, 1)
        if (DESIRED_DISTANCE - variance) <= distance <= (DESIRED_DISTANCE + variance) and (0 - variance) <= angle <= (0 + variance):
            cur_state = State.stop
        elif distance < DESIRED_DISTANCE and abs(angle) >= (0 + variance):
            cur_state = State.reverse
    elif cur_state == State.reverse:
        speed = rc_utils.remap_range(distance, 0, 700, -1, 0)
        #speed = rc_utils.remap_range(distance, 0, 700, -1, 0)
        #speed = rc_utils.clamp(speed, -1, 0)
        if distance > FORWARD_DISTANCE:
            cur_state = State.align
    elif cur_state == State.stop:
        speed = 0
        if distance > (DESIRED_DISTANCE + variance):
            cur_state = State.align
        elif distance < BACK_DISTANCE:
            cur_state = State.reverse
    if speed < 0:
        angle *= -1
    rc.drive.set_speed_angle(speed, angle)
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
