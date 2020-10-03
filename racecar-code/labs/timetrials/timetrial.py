"""
Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Final Challenge - Time Trials
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
# Colors
########################################################################################
BLUE = ((90, 100, 100), (120, 255, 255))  # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
PURPLE = ((130, 50, 50), (155, 255, 255)) # The HSV range for the color purple
ORANGE = ((10, 50, 50), (20, 255, 255)) # The HSV range for the color orange

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

COLOR_PRIORITY = [BLUE, RED, GREEN, ORANGE, PURPLE] # Starting color priority
USED_COLORS = (RED, GREEN, BLUE, ORANGE, PURPLE)
MIN_CONTOUR_AREA = 200 # Min countour area to recognize
CROP_FLOOR = ((470, 0), (rc.camera.get_height(), rc.camera.get_width())) # Crop everything but the floor
CROP_AR_LEFT = ((40, 0), (((2 * rc.camera.get_height())) // 3, rc.camera.get_width() // 2))
CROP_AR_RIGHT = ((40, rc.camera.get_width() // 2), (((2 * rc.camera.get_height())) // 3, rc.camera.get_width()))
CROP_LEFT_FLOOR = ((300, 0), (rc.camera.get_height(), rc.camera.get_width() // 2))
CROP_RIGHT_FLOOR = ((300, rc.camera.get_width() // 2), (rc.camera.get_height(), rc.camera.get_width()))
COLOR = ""

speed: float = 0.0
angle: float = 0.0
contour_center = None
contour_area = 0
contour_area_ar = 0
corners = 0
counter = 0
center = 0
x = 0

class State(IntEnum):
    line_follow = 0
    line_center_fast = 1
    line_center_split = 2
    line_center_turn = 3
    slalom_red = 4
    slalom_blue = 5
    wall_follow_beg = 6
    wall_follow_no = 7
    finish_sprint = 8
    reverse = 9
cur_state: State = State.line_follow

class Direction(IntEnum):
    """
    AR marker direction
    """
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

########################################################################################
# Functions
########################################################################################

def update_contour(image, min_contour_area):
    global contour_center
    global contour_area
    global COLOR

    if image is None:
        contour_center = None
        countour_area = 0
    else:
        for color in COLOR_PRIORITY:
            contours = rc_utils.find_contours(image, color[0], color[1])
            contour = rc_utils.get_largest_contour(contours, min_contour_area)
            if contour is not None:
                COLOR = color
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                rc_utils.draw_contour(image, contour)
                rc_utils.draw_circle(image, contour_center)
                break
            else:
                contour_center = None
                contour_area = 0

def update_contour_ar(image, min_contour_area):
    global contour_area_ar
    global COLOR

    if image is None:
        countour_area_ar = 0
    else:
        for color in COLOR_PRIORITY:
            contours_ar = rc_utils.find_contours(image, color[0], color[1])
            contour_ar = rc_utils.get_largest_contour(contours_ar, min_contour_area)
            if contour_ar is not None:
                COLOR = color
                contour_area_ar = rc_utils.get_contour_area(contour_ar)

                rc_utils.draw_contour(image, contour_ar)
                break
            else:
                contour_area_ar = 0
    
    #rc.display.show_color_image(image)

def update_ar(image):
    global corners 
    global center

    ar_directions = []
    corners, ids = rc_utils.get_ar_markers(image)
    image = rc_utils.draw_ar_markers(image, corners, ids, (0, 255, 0))
    for i in range(len(corners)):
        ar_directions.append(rc_utils.get_ar_direction(corners[i]))
    if len(ar_directions) is not 0:
        center = (int((corners[0][0][0][0] + corners[0][0][3][0]) // 2), int((corners[0][0][0][1] + corners[0][0][1][1]) // 2))
    return ar_directions

def start():
    """
    This function is run once every time the start button is pressed
    """
    global speed
    global angle
    global cur_state
    global counter

    cur_state = State.line_follow
    counter = 0

    # Have the car begin at a stop
    rc.drive.stop()
    color_image = rc.camera.get_color_image()
    ar_image = color_image
    depth_image = rc.camera.get_depth_image()
    scan = rc.lidar.get_samples()

    # Print start message
    print(">> Final Challenge - Time Trials")


def update():
    """
    After start() is run, this function is run every frame until the back button
    is pressed
    """
    global speed
    global angle 
    global cur_state
    global counter
    global corners
    global center
    global x

    color_image = rc.camera.get_color_image()
    ar_image = color_image
    depth_image = rc.camera.get_depth_image()
    scan = rc.lidar.get_samples()
    forward_dist = rc_utils.get_lidar_average_distance(scan, 0, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, 270, 40)
    right_dist = rc_utils.get_lidar_average_distance(scan, 90, 40)
    ar_directions = update_ar(ar_image)

    color_image_copy = np.copy(color_image)
    color_image_copy = rc_utils.crop(color_image_copy, CROP_FLOOR[0], CROP_FLOOR[1])
    #rc.display.show_color_image(color_image_copy)

    left_floor = np.copy(color_image)
    right_floor = np.copy(color_image)
    left_floor = rc_utils.crop(left_floor, CROP_LEFT_FLOOR[0], CROP_LEFT_FLOOR[1])
    right_floor = rc_utils.crop(right_floor, CROP_RIGHT_FLOOR[0], CROP_RIGHT_FLOOR[1])
    floor = rc_utils.crop(color_image, CROP_LEFT_FLOOR[0], CROP_RIGHT_FLOOR[1])
    rc.display.show_color_image(floor)
     
    ar_image_left = 0
    ar_image_right = 0
    distance = 600
    counter += rc.get_delta_time()
    if cur_state == State.line_follow:
        if len(ar_directions) is not 0:
            # if center[0] < rc.camera.get_height() and center[1] < rc.camera.get_width():
            #     distance = rc_utils.get_pixel_average_distance(depth_image, center)
            #     print(distance)
            for direction in ar_directions:
                left_color = COLOR_PRIORITY[0]
                right_color = COLOR_PRIORITY[4]
                if direction == Direction.LEFT:
                    ar_image_left = np.copy(ar_image)
                    ar_image_left = rc_utils.crop(ar_image_left, CROP_AR_LEFT[0], CROP_AR_LEFT[1])
                    update_contour_ar(ar_image_left, 500)
                    left_color: tuple = COLOR
                elif direction == Direction.RIGHT:
                    ar_image_right = np.copy(ar_image)
                    ar_image_right = rc_utils.crop(ar_image_right, CROP_AR_RIGHT[0], CROP_AR_RIGHT[1])
                    update_contour_ar(ar_image_right, 500)
                    right_color: tuple = COLOR
                elif direction == Direction.UP:
                    line_center_image = np.copy(ar_image)
                    line_center_image = rc_utils.crop(line_center_image, CROP_AR_RIGHT[0], CROP_AR_RIGHT[1])
                    update_contour_ar(line_center_image, 500)
                    #rc.display.show_color_image(line_center_image)
                    print(contour_area)
                    if contour_area < 600:
                        cur_state = State.line_center_fast
                        print("State changed")
            COLOR_PRIORITY.clear()
            COLOR_PRIORITY.append(left_color)
            for color in USED_COLORS:
                if color != left_color or color != right_color:
                    COLOR_PRIORITY.append(color)
            COLOR_PRIORITY.append(right_color)
        update_contour(color_image_copy, MIN_CONTOUR_AREA)
        if counter > 5 and COLOR_PRIORITY[0] != COLOR:
            COLOR_PRIORITY[0] = COLOR
        if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, (3 * rc.camera.get_width()) // 4, -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)
        else:
            angle = 0
        if abs(angle) < 0.15 and counter <= 10:
            speed = 0.25
        else:
            speed = 0.5
 
    elif cur_state == State.line_center_fast:
        speed = 0.75
        #print(forward_dist)
        # center = (int((corners[0][0][0][0] + corners[0][0][3][0]) // 2), int((corners[0][0][0][1] + corners[0][0][1][1]) // 2))
            # if center[0] < rc.camera.get_height() and center[1] < rc.camera.get_width():
            #     distance = rc_utils.get_pixel_average_distance(depth_image, center)
            #     if distance < 30:
        if x == 0:
            line_center_color: tuple = COLOR
            COLOR_PRIORITY.clear()
            COLOR_PRIORITY.append(line_center_color)
            if line_center_color == ORANGE:
                COLOR_PRIORITY.append(PURPLE)
            else:
                COLOR_PRIORITY.append(ORANGE)
        x = 1
        counter = 0
        #if contour_area != 0:
        update_contour(left_floor, 10)
        contour_area_left = contour_area
        update_contour(right_floor, 10)
        contour_area_right = contour_area
        print("Left" + str(contour_area_left))
        print("Right" + str(contour_area_right))
        angle = rc_utils.remap_range(contour_area_right - contour_area_left, -15000, 15000, -0.575, 0.575)
        if len(ar_directions) is not 0 and forward_dist < 160:
            cur_state = State.line_center_split
            counter = 0
        # elif forward_dist < 15:
        #     cur_state = State.line_center_turn
    #     if len(ar_directions) is not 0:
    #         center = (int((corners[0][0][0][0] + corners[0][0][3][0]) // 2), int((corners[0][0][0][1] + corners[0][0][1][1]) // 2))
    #         if center[0] < rc.camera.get_height() and center[1] < rc.camera.get_width():
    #             distance = rc_utils.get_pixel_average_distance(depth_image, center)
    #             if 0 < distance < 1:
    #                 cur_state = State.line_center_split
    
    elif cur_state == State.line_center_split:
        print(left_dist)
        print(right_dist)
        if len(ar_directions) is not 0:
            for direction in ar_directions:
                    if direction == Direction.LEFT:
                        angle = -0.25
                    elif direction == Direction.RIGHT:
                        angle = 0.25
        if counter >= 1: 
            cur_state = State.line_center_fast
            print("Changed State")

    
    angle = rc_utils.clamp(angle, -1, 1)
    speed = rc_utils.clamp(speed, -1, 1)
    rc.drive.set_speed_angle(speed, angle)

    print(
        f"State: {cur_state.name}, speed: {speed:.2f}, angle: {angle:2f}"
    )


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()