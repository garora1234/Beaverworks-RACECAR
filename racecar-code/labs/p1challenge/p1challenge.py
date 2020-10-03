"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Phase 1 Challenge - Cone Slaloming
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
MIN_CONTOUR_AREA = 170
LEFT_COL = int(rc.camera.get_width() * 0.3)
RIGHT_COL = int(rc.camera.get_width() * 0.7)
BOTTOM_ROW = int(rc.camera.get_height() * 0.65)
BLUE = ((100, 150, 150), (120, 255, 255))  # The HSV range for the color blue
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
DESIRED_DISTANCE = 50
variance: float = 1
speed: float = 0.0
angle: float = 0.0
contour_center = None
contour_area = 0
counter = 0
passing_speed = 0.5
passing_red = (rc.camera.get_width() // 2) + 300
passing_blue = (rc.camera.get_width() // 2) - 300
time1 = 1.25
time2 = 1
c = 0

class State(IntEnum):
    approach_red = 0
    approach_blue = 1
    red = 2
    blue = 3
cur_state: State = State.approach_red   

color = True
########################################################################################
# Functions
########################################################################################

def update_contour(HSV):
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
        # Find all of the contours
        contours = rc_utils.find_contours(image, HSV[0], HSV[1])

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
    global counter
    global color
    global c

    cur_state = State.approach_red
    color = True
    # Have the car begin at a stop
    rc.drive.stop()
    speed = 0
    angle = 0
    c = 0
    depth_image = rc.camera.get_depth_image()
    # Print start message
    print(">> Phase 1 Challenge: Cone Slaloming")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global cur_state
    global counter
    global time1
    global color
    global c
    global MIN_CONTOUR_AREA

    # TODO: Slalom between red and blue cones.  The car should pass to the right of
    # each red cone and the left of each blue cone.
    # speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    # LeftJoy = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    # if speed == 0:
    #     speed = - rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    # #print("Driving...")
    # rc.drive.set_speed_angle(speed, LeftJoy[0])
    if color:
        update_contour(RED)
    else:
        update_contour(BLUE)
    
    depth_image = rc.camera.get_depth_image()
    depth_image_cropped = rc_utils.crop(depth_image, (0, LEFT_COL), (BOTTOM_ROW, RIGHT_COL))
    closest_point = rc_utils.get_closest_pixel(depth_image_cropped)
    distance = rc_utils.get_pixel_average_distance(depth_image_cropped, closest_point)
    #rc.display.show_depth_image(depth_image_cropped, points = [closest_point])
    d = 0
    if c < 3:
        d = 700
    else:
        d = 200
        time1 = 1
        MIN_CONTOUR_AREA = 200
    if cur_state == State.approach_blue or cur_state == State.approach_red:
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        angle = rc_utils.clamp(angle, -1, 1)
        speed = rc_utils.remap_range(distance, 0, d, 0, 1)
        speed = rc_utils.clamp(speed, 0, 1)
        if (DESIRED_DISTANCE - variance) <= distance <= (DESIRED_DISTANCE + variance):
            counter = 0
            if cur_state == State.approach_red:
                counter = 0
                cur_state = State.red
            elif cur_state == State.approach_blue:
                counter = 0
                cur_state = State.blue
    elif cur_state == State.blue or cur_state == State.red:
        counter += rc.get_delta_time()
        speed = passing_speed
        if cur_state == State.red:
            color = False
            if counter < time1:
                angle = rc_utils.remap_range(passing_red, 0, rc.camera.get_width(), -1, 1)
            else:
                angle = rc_utils.remap_range(passing_blue, 0, rc.camera.get_width(), -1, 1)
                if contour_center is not None:
                    print(contour_area)
                    cur_state = State.approach_blue
        elif cur_state == State.blue:
            color = True
            if counter < time1: 
                angle = rc_utils.remap_range(passing_blue, 0, rc.camera.get_width(), -1, 1)
            else:
                angle = rc_utils.remap_range(passing_red, 0, rc.camera.get_width(), -1, 1)
                update_contour(RED)
                if contour_center is not None:
                    c += 1
                    print(contour_area)
                    cur_state = State.approach_red
    rc.drive.set_speed_angle(speed, angle)
########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
