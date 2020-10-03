"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 3B - Depth Camera Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
from enum import IntEnum
from simple_pid import PID

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here
MIN_CONTOUR_AREA = 30
LEFT_COL = int(rc.camera.get_width() * 0.3)
RIGHT_COL = int(rc.camera.get_width() * 0.6)
BOTTOM_ROW = int(rc.camera.get_height() * 0.65)
ORANGE = ((10, 100, 100), (20, 255, 255))
DESIRED_DISTANCE = 30
BACK_DISTANCE = DESIRED_DISTANCE * 0.4
FORWARD_DISTANCE = DESIRED_DISTANCE * 0.2
variance: float = 1
speed: float = 0.0
angle: float = 0.0
contour_center = None
contour_area = 0
kP = 0.25
kI = 0.1
kD = 0.1
pid = PID(kP, kI, kD, setpoint = DESIRED_DISTANCE)

class State(IntEnum):
    align = 0
    stop = 1
    reverse = 2
    search = 3
cur_state: State = State.align
########################################################################################
# Functions
########################################################################################

def update_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Find all of the orange contours
        contours = rc_utils.find_contours(image, ORANGE[0], ORANGE[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(image, contour)
            rc_utils.draw_circle(image, contour_center)

        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
        rc.display.show_color_image(image)


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
    depth_image = rc.camera.get_depth_image()

    # Print start message
    print(">> Lab 3B - Depth Camera Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state
    global variance

    update_contour()
    depth_image = rc.camera.get_depth_image()
    depth_image_cropped = rc_utils.crop(depth_image, (0, LEFT_COL), (BOTTOM_ROW, RIGHT_COL))
    closest_point = rc_utils.get_closest_pixel(depth_image_cropped)
    distance = rc_utils.get_pixel_average_distance(depth_image_cropped, closest_point)

    # TODO: Park the car 30 cm away from the closest orange cone.
    # Use both color and depth information to handle cones of multiple sizes.
    # You may wish to copy some of your code from lab2b.py
    if cur_state == State.align:
        if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            #speed = pid(distance)
            #speed = rc_utils.clamp(speed, 0, 1)
            speed = rc_utils.remap_range(distance, 0, 700, 0, 1)
            rc.drive.set_speed_angle(speed, angle)
            if (DESIRED_DISTANCE - variance) <= distance <= (DESIRED_DISTANCE + variance):
                cur_state = State.stop
            elif distance < BACK_DISTANCE:
                cur_state = State.reverse
        else:
            cur_state = State.search
    elif cur_state == State.reverse:
        if contour_center is not None:
            #speed = pid(distance)
            #speed = rc_utils.clamp(speed, -1, 0)
            speed = -0.25
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            rc.drive.set_speed_angle(speed, angle)
            if distance > FORWARD_DISTANCE:
                cur_state = State.align
        else:
            cur_state = State.search
    elif cur_state == State.stop:
        if contour_center is not None:
            speed = 0
            rc.drive.stop()
            if distance > (DESIRED_DISTANCE + variance):
                cur_state = State.align
            elif distance < BACK_DISTANCE:
                cur_state = State.reverse
        else:
            cur_state = State.search
    elif cur_state == State.search:
        if contour_center is not None:
            if (DESIRED_DISTANCE - variance) <= distance <= (DESIRED_DISTANCE + variance):
                cur_state = State.stop
            elif distance < BACK_DISTANCE:
                cur_state = State.reverse
            else:
                cur_state = State.align
        else:
            rc.drive.stop() #change after everything else works
    if speed < 0:
        angle *= -1
        
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
