"""
Copyright Harvey Mudd College
MIT License
Fall 2019

Lab 4A - IMU: Roll Prevention
"""

################################################################################
# Imports
################################################################################

import sys
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

import cv2 as cv
import numpy as np
import math


################################################################################
# Global variables
################################################################################

rc = racecar_core.create_racecar()
ang_vel = 0
lin_acc = 0
################################################################################
# Functions
################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global ang_vel
    global lin_acc

    # Begin at a full stop
    rc.drive.stop()
    ang_vel = rc.physics.get_angular_velocity()
    lin_acc = rc.physics.get_linear_acceleration()

    # Print start message
    print(
        ">> Lab 4A - IMU: Roll Prevention\n"
        "\n"
        "Controlls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global ang_vel
    global lin_acc

    ang_vel = rc.physics.get_angular_velocity()
    lin_acc = rc.physics.get_linear_acceleration()

    # Calculate speed from triggers
    forwardSpeed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    backSpeed = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = forwardSpeed - backSpeed

    # Calculate angle from left joystick
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # TODO (warmup): Prevent the car from turning too abruptly using the IMU
    if abs(ang_vel[0]) > 0.2 and abs(ang_vel[1]) > 0.1 and abs(ang_vel[2]) > 0.2:
        speed = -speed
        angle = -angle
    rc.drive.set_speed_angle(speed, angle)


################################################################################
# Do not modify any code beyond this point
################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
