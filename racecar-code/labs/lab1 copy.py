"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 1 - Driving in Shapes
"""

########################################################################################
# Imports
########################################################################################

import sys

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Put any global variables here
counter = 0
isDriving = False
isSquare = False
isCircle = False
isFigureEight = False
isTriangle = False
LeftJoy = (0,0)

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global counter
    global isDriving
    global isSquare
    global isCircle
    global isFigureEight
    global isTriangle
    global LeftJoy
    # Begin at a full stop
    rc.drive.stop()

    # Print start message
    # TODO (main challenge): add a line explaining what the Y button does
    print(
        ">> Lab 1 - Driving in Shapes\n"
        "\n"
        "Controls:\n"
        "   Right trigger = accelerate forward\n"
        "   Left trigger = accelerate backward\n"
        "   Left joystick = turn front wheels\n"
        "   A button = drive in a circle\n"
        "   B button = drive in a square\n"
        "   X button = drive in a figure eight\n"
        "   Y button = drive in a triangle\n"
    )


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    # Initialize variables
    global counter
    global isDriving
    global isSquare
    global isCircle
    global isFigureEight
    global isTriangle
    global LeftJoy
    # Implement acceleration and steering
    speed = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    LeftJoy = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    if speed == 0:
        speed = - rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    #print("Driving...")
    rc.drive.set_speed_angle(speed, LeftJoy[0])

    # Drive in a circle if button A pressed
    if rc.controller.was_pressed(rc.controller.Button.A):
        print("Driving in a circle...")
        counter = 0
        isCircle = True
    if isCircle:
        counter += rc.get_delta_time()   
        rc.drive.set_speed_angle(1, 1)

    # Drive in a square when the B button is pressed
    if rc.controller.was_pressed(rc.controller.Button.B):
        print("Driving in a square...")
        counter = 0
        isSquare = True    
    if isSquare:
    # rc.get_delta_time() gives the time in seconds since the last time
    # the update function was called
        counter += rc.get_delta_time()
        if counter < 1:
            # Drive forward at full speed for one second
            rc.drive.set_speed_angle(1, 0)
        elif counter < 2.52:
            # Turn left at full speed for the next second
            rc.drive.set_speed_angle(1, 1)
        else:
            # Continue infinitely
            counter = 0
        
    # Drive in a figure eight when the X button is pressed
    if rc.controller.was_pressed(rc.controller.Button.X):
        print("Driving in a figure eight...")
        counter = 0
        isFigureEight = True
    if isFigureEight:
        counter += rc.get_delta_time()
        if counter < 6.5:
            rc.drive.set_speed_angle(1,1)
        elif counter < 13:
            rc.drive.set_speed_angle(1,-1)
        else:
            counter = 0
    # Drive in a shape of your choice when the Y button is pressed
    if rc.controller.was_pressed(rc.controller.Button.Y):
        print("Driving in a triangle...")
        counter = 0
        isTriangle = True
    if isTriangle:
        counter += rc.get_delta_time()
        if counter < 1:
            rc.drive.set_speed_angle(1,0)
        elif counter < 2.85:
            rc.drive.set_speed_angle(1,1)
        else:
            counter = 0

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()