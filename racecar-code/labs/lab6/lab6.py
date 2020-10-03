"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Lab 6 - Sensor Fusion
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv
import numpy as np

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

velocity = 0
prev_distance = 0 # depth camera
pre_dist = 0 # LIDAR
v1 = 0 # IMU
v2 = 0 # depth
v3 = 0 # LIDAR

########################################################################################
# Functions
########################################################################################


def start():
    """
    This function is run once every time the start button is pressed
    """
    global prev_distance
    global velocity
    global pre_dist

    # Have the car begin at a stop
    rc.drive.stop()

    depth_image = rc.camera.get_depth_image()
    prev_distance = rc_utils.get_depth_image_center_distance(depth_image)

    scan = rc.lidar.get_samples()
    pre_dist = rc_utils.get_lidar_average_distance(scan, 0, 10)

    # Print start message
    print(">> Lab 6 - LIDAR Safety Stop")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global prev_distance
    global pre_dist
    global velocity
    global v1
    global v2
    global v3

    # Use the triggers to control the car's speed
    rt = rc.controller.get_trigger(rc.controller.Trigger.RIGHT)
    lt = rc.controller.get_trigger(rc.controller.Trigger.LEFT)
    speed = rt - lt

    # Use the left joystick to control the angle of the front wheels
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]

    # TODO: Estimate the car's speed with at least 3 unique methods
    accel = rc.physics.get_linear_acceleration()
    v1 = accel[2] * rc.get_delta_time()
    print("v1: " + str(v1))

    depth_image = rc.camera.get_depth_image()
    distance = rc_utils.get_pixel_average_distance(depth_image, [rc.camera.get_width() // 2, rc.camera.get_height() // 2], 5)
    if distance > 0:
        frame_speed = (prev_distance - distance) / rc.get_delta_time()
        v2 = 0.2 * (frame_speed - v2)
        prev_distance = distance
        print("v2: " + str(v2))
    
    scan = rc.lidar.get_samples()
    front_dist = rc_utils.get_lidar_average_distance(scan, 0, 10)
    if front_dist > 0:
        f_speed = (pre_dist - front_dist) / rc.get_delta_time()
        v3 = 0.2 * (f_speed - v3)
        pre_dist = front_dist
    # TODO: Fuse these sources into a single velocity measurement
    if 0 < v1 < 1 and 0 < v2 < 1 and 0 < v3 < 1:
        velocity = (0.5 * v1) + (0.2 * v2) + (0.3 * v3)
        print(velocity)
    # TODO: Prevent the car from traveling over 0.5 m/s
    if speed >= 0.5:
        speed = 0.4
    if velocity >= 0.4:
        speed -= 0.1
    elif velocity >= 0.46:
        rc.drive.stop()
    rc.drive.set_speed_angle(speed, angle)


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
