"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 2B - Color Image Cone Parking
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np
import random
from simple_pid import PID
from enum import IntEnum

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# >> Constants
# The smallest contour we will recognize as a valid contour
MIN_CONTOUR_AREA = 30
DESIRED_CONTOUR_AREA = 26500
MAX_CONTOUR_AREA = 51000

# The HSV range for the color orange, stored as (hsv_min, hsv_max)
ORANGE = ((10, 100, 100), (20, 255, 255))

# >> Variables
counter = 0
speed: float = 0.0  # The current speed of the car
angle: float = 0.0  # The current angle of the car's wheels
contour_center = None  # The (pixel row, pixel column) of contour
contour_area = 0  # The area of contour
kP = 0.25
kI = 0.025
kD = 0.45

pid = PID(kP, kI, kD, setpoint=DESIRED_CONTOUR_AREA)


class State(IntEnum):
    align = 0
    stop = 1
    reverse = 2
    search = 3
cur_state: State = State.align

# ########################################################################################
# # Functions
# ########################################################################################


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
    global counter
    global cur_state
    cur_state = State.align
    global pid
    global kP
    global kI
    global kD
    pid = PID(kP, kI, kD, setpoint=DESIRED_CONTOUR_AREA)
    pid.output_limits = (-1, 1)


    # Initialize variables
    speed = 0
    angle = 0

    # Set initial driving speed and angle
    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

    # Print start message
    print(">> Lab 2B - Color Image Cone Parking")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle
    global counter
    global cur_state
    global pid

    # Search for contours in the current color image
    update_contour()

    # Park the car 30 cm away from the closest orange cone
    if cur_state == State.align:
        if contour_center is not None:   
            angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
            speed = pid(contour_area)
            #speed = rc_utils.remap_range(contour_area, MIN_CONTOUR_AREA, MAX_CONTOUR_AREA, 0, 1)
            rc.drive.set_speed_angle(speed, angle)
            if contour_area >= DESIRED_CONTOUR_AREA - 500 and contour_area <= DESIRED_CONTOUR_AREA + 500:
                cur_state = State.stop
            elif contour_area > DESIRED_CONTOUR_AREA - 500:
                cur_state = State.reverse
        else:
            cur_state = State.search
    if cur_state == State.stop:
        if contour_center is not None:
            speed = 0
            rc.drive.set_speed_angle(speed, angle)
            if contour_area < DESIRED_CONTOUR_AREA + 500:
                cur_state = State.align
            elif contour_area > DESIRED_CONTOUR_AREA - 500:
                cur_state = State.reverse
        else:
            cur_state = State.search
    if cur_state == State.reverse:
        if contour_center is not None:
            speed = -0.6
            rc.drive.set_speed_angle(speed, angle)
            if contour_area >= DESIRED_CONTOUR_AREA - 500 and contour_area <= DESIRED_CONTOUR_AREA + 500:
                cur_state = State.stop
            elif contour_area < DESIRED_CONTOUR_AREA + 500:
                cur_state = State.align
        else:
            cur_state = State.search
    if cur_state == State.search:
        if counter < 40:
            angle = -random.random()
        elif counter < 80:
            angle = random.random()
        else:
            counter = 0
        speed = 0.25
        rc.drive.set_speed_angle(speed, angle)
        if contour_center is not None:
            if contour_area >= DESIRED_CONTOUR_AREA - 500 and contour_area <= DESIRED_CONTOUR_AREA + 500:
                cur_state = State.stop
            elif contour_area < DESIRED_CONTOUR_AREA - 500:
                cur_state = State.align
            else:
                cur_state = State.reverse
        counter += rc.get_delta_time()

    # Print the current speed and angle when the A button is held down
    if rc.controller.is_down(rc.controller.Button.A):
        print("Speed:", speed, "Angle:", angle)

    # Print the center and area of the largest contour when B is held down
    if rc.controller.is_down(rc.controller.Button.B):
        if contour_center is None:
            print("No contour found")
        else:
            print("Center:", contour_center, "Area:", contour_area)


def update_slow():
    """
    After start() is run, this function is run at a constant rate that is slower
    than update().  By default, update_slow() is run once per second
    """
    # Print a line of ascii text denoting the contour area and x position
    if rc.camera.get_color_image() is None:
        # If no image is found, print all X's and don't display an image
        print("X" * 10 + " (No image) " + "X" * 10)
    else:
        # If an image is found but no contour is found, print all dashes
        if contour_center is None:
            print("-" * 32 + " : area = " + str(contour_area))

        # Otherwise, print a line of dashes with a | indicating the contour x-position
        else:
            s = ["-"] * 32
            s[int(contour_center[1] / 20)] = "|"
            print("".join(s) + " : area = " + str(contour_area))


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()