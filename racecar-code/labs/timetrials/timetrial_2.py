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
BLUE = ((90, 100, 100), (110, 255, 255))  # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
PURPLE = ((135, 100, 100), (150, 255, 255)) # The HSV range for the color purple
ORANGE = ((10, 50, 50), (20, 255, 255)) # The HSV range for the color orange

########################################################################################
# Global variables
########################################################################################
rc = racecar_core.create_racecar()

COLOR_PRIORITY = [BLUE, RED, GREEN, ORANGE, PURPLE] # Starting color priority
USED_COLORS = (RED, GREEN, BLUE, ORANGE, PURPLE)
MIN_CONTOUR_AREA = 200
CROP_FLOOR = ((465, 0), (rc.camera.get_height(), rc.camera.get_width())) # Crop everything but the floor
CROP_AR_LEFT = ((40, 0), (((2 * rc.camera.get_height())) // 3, rc.camera.get_width() // 2))
CROP_AR_RIGHT = ((40, rc.camera.get_width() // 2), (((2 * rc.camera.get_height())) // 3, rc.camera.get_width()))
CROP_LEFT_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width() // 2))
CROP_RIGHT_FLOOR = ((360, rc.camera.get_width() // 2), (rc.camera.get_height(), rc.camera.get_width()))
COLOR = ""
COLOR_TURN = ""
DESIRED_DISTANCE = 35
CONE_DISTANCE = 70
TURN_DISTANCE = 110
ACCEL_DISTANCE = 120
LEFT_COL = int(rc.camera.get_width() * 0.3)
RIGHT_COL = int(rc.camera.get_width() * 0.7)
BOTTOM_ROW = int(rc.camera.get_height() * 0.65)

speed: float = 0.0
angle: float = 0.0
contour_center = None
contour_area = 0
contour_area_ar = 0
corners = 0
counter = 0
center = 0
x = False

# Speeds
MAX_ALIGN_SPEED = 0.8
MIN_ALIGN_SPEED = 0.4
PASS_SPEED = 0.5
FIND_SPEED = 0.2
REVERSE_SPEED = -0.2
NO_CONES_SPEED = 0.4

# Times
REVERSE_BRAKE_TIME = 0.25
SHORT_PASS_TIME = 1.0
LONG_PASS_TIME = 1.2

# Cone finding parameters
MIN_CONTOUR_AREA = 100
MAX_DISTANCE = 250
REVERSE_DISTANCE = 50
STOP_REVERSE_DISTANCE = 60

CLOSE_DISTANCE = 30
FAR_DISTANCE = 120

# >> Variables

counter = 0
red_center = None
red_distance = 0
prev_red_distance = 0
blue_center = None
blue_distance = 0
prev_blue_distance = 0

class State(IntEnum):
    line_follow = 0
    line_center_fast = 1
    line_center_split = 2
    line_center_turn = 3
    slalom = 4
    wall_follow_accel = 5
    wall_follow_turn = 6
    wall_follow_pass_left = 7
    wall_follow_pass_right = 8
    finish_sprint = 9
cur_state: State = State.line_follow

class Direction(IntEnum):
    """
    AR marker direction
    """
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Mode(IntEnum):
    cone_align = 0
    cone_pass = 1
    cone_blue = 2
    cone_red = 3
    reverse = 4
    no_cones = 5
cur_mode = Mode.cone_align
########################################################################################
# Functions
########################################################################################

def update_contour(image, min_contour_area):
    global contour_center
    global contour_area
    global COLOR_TURN

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        for color in COLOR_PRIORITY:
            contours = rc_utils.find_contours(image, color[0], color[1])
            contour = rc_utils.get_largest_contour(contours, min_contour_area)
            if contour is not None:
                COLOR_TURN = color
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
        contour_area_ar = 0
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
    global cur_mode

    cur_state = State.slalom
    counter = 0
    cur_mode = Mode.cone_align

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
    global cur_mode


    color_image = rc.camera.get_color_image()
    ar_image = color_image
    depth_image = rc.camera.get_depth_image()
    
    scan = rc.lidar.get_samples()
    forward_dist = rc_utils.get_lidar_average_distance(scan, 0, 5)
    back_dist = rc_utils.get_lidar_average_distance(scan, 180, 5)
    top_right_dist = rc_utils.get_lidar_average_distance(scan, 40, 20)
    bot_right_dist = rc_utils.get_lidar_average_distance(scan, 140, 20)
    top_left_dist = rc_utils.get_lidar_average_distance(scan, 300, 20)
    bot_left_dist = rc_utils.get_lidar_average_distance(scan, 240, 20)
    right_dist = rc_utils.get_lidar_average_distance(scan, 90, 10)
    left_dist = rc_utils.get_lidar_average_distance(scan, 270, 10)
    ar_directions = update_ar(ar_image)

    floor = np.copy(color_image)
    floor = rc_utils.crop(floor, CROP_FLOOR[0], CROP_FLOOR[1])

    left_floor = np.copy(color_image)
    right_floor = np.copy(color_image)
    left_floor = rc_utils.crop(left_floor, CROP_LEFT_FLOOR[0], CROP_LEFT_FLOOR[1])
    right_floor = rc_utils.crop(right_floor, CROP_RIGHT_FLOOR[0], CROP_RIGHT_FLOOR[1])

    cones = np.copy(color_image)
    rc.display.show_color_image(cones)
    cone_depth = np.copy(depth_image)
    cone_depth = rc_utils.crop(cone_depth, (0, LEFT_COL), (BOTTOM_ROW, RIGHT_COL))
    cone_closest_point = rc_utils.get_closest_pixel(cone_depth)
    cone_distance = rc_utils.get_pixel_average_distance(cone_depth, cone_closest_point)

    ar_image_left = 0
    ar_image_right = 0
    counter += rc.get_delta_time()
    if cur_state == State.line_follow:
        if len(ar_directions) is not 0:
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
                    x = True
            COLOR_PRIORITY.clear()
            COLOR_PRIORITY.append(left_color)
            for color in USED_COLORS:
                if color != left_color or color != right_color:
                    COLOR_PRIORITY.append(color)
            COLOR_PRIORITY.append(right_color)
        update_contour(floor, MIN_CONTOUR_AREA)
        if contour_center is not None:
            angle = rc_utils.remap_range(contour_center[1], 0, (3 * rc.camera.get_width()) // 4, -1, 1)
            angle = rc_utils.clamp(angle, -1, 1)
        else:
            angle = 0
        if counter >= 13 and 0 < top_left_dist < 200 and x == True:
            counter = 0
            cur_state = State.line_center_fast
            print("State changed")
        if abs(angle) < 0.25:
            speed = 1
        else:
            speed = 0.75
        
 
    elif cur_state == State.line_center_fast:
        speed = 0.75
        angle = 0
        line_center_color = COLOR
        COLOR_PRIORITY.clear()
        COLOR_PRIORITY.append(line_center_color)
        if line_center_color == ORANGE:
            COLOR_PRIORITY.append(PURPLE)
        else:
            COLOR_PRIORITY.append(ORANGE)
        update_contour(left_floor, 1)
        contour_area_left = contour_area
        update_contour(right_floor, 1)
        contour_area_right = contour_area
        if contour_area_right != 0 and contour_area_left != 0: 
            angle = rc_utils.remap_range(contour_area_left - contour_area_right, -2000, 2000, -1, 1)
        elif contour_area_left != 0 and counter >= 3:
            angle = 0.05
        elif contour_area_right != 0 and counter >= 3:
            angle = -0.05
        else:
            angle = 0
        if len(ar_directions) is not 0 and 0 < forward_dist < 140 and counter >= 3:
            cur_state = State.line_center_split
        elif COLOR_TURN == COLOR_PRIORITY[1] and counter >= 10:
            if len(ar_directions) is not 0:
                if 0 < left_dist < 200:
                    counter = 0
                    x = False
                    cur_state = State.slalom
                else:
                    speed = 1
                    angle = 0.5
            else:
                cur_state = State.line_center_turn
        if 0.15 < abs(angle) < 0.5:
            speed = 0.75
        elif abs(angle) > 0.5:
            speed = 0.5
        else:
            speed = 1
    
    elif cur_state == State.line_center_split:
        if len(ar_directions) is not 0:
            for direction in ar_directions:
                    if direction == Direction.LEFT:
                        angle = -0.25
                    elif direction == Direction.RIGHT:
                        angle = 0.25
        elif counter >= 1.125: 
            cur_state = State.line_center_fast
            print("Changed State")

    elif cur_state == State.line_center_turn:
        update_contour(left_floor, 1)
        contour_area_left = contour_area
        update_contour(right_floor, 1)
        contour_area_right = contour_area
        if contour_area_left != 0 and contour_area_right != 0:
            speed = 0.5
            angle = rc_utils.remap_range(contour_area_left - contour_area_right, -1500, 1500, -1, 1)
            if COLOR_TURN == COLOR_PRIORITY[0] and contour_area_right > 200 and contour_area_left > 200:
                cur_state = State.line_center_fast
        else:
            speed = 0.75
            angle = -1
        print("Left" + str(contour_area_left))
        print("Right" + str(contour_area_right))
    
    elif cur_state == State.slalom:
        speed = 1
        angle = -0.087
        if abs(rc.physics.get_linear_acceleration()[2]) > 1:
            angle = 0
        if len(ar_directions) is not 0:
            x = True
        if 0 < left_dist < 100 and 0 < right_dist < 100 and x == True:
            counter = 0
            cur_state = State.wall_follow_accel 

    elif cur_state == State.wall_follow_accel:
        speed = 0.5
        right_dif = top_right_dist - bot_right_dist
        left_dif = top_left_dist - bot_left_dist
        if right_dist > left_dist:
            angle = rc_utils.remap_range(left_dif, -DESIRED_DISTANCE, DESIRED_DISTANCE, 1, -1)
        else:
            angle = rc_utils.remap_range(right_dif, -DESIRED_DISTANCE, DESIRED_DISTANCE, -1, 1)
        if abs(angle) < 0.06:
            speed = 1
        elif 0.06 <= abs(angle) <= 0.15:
            speed = 0.75
        if len(ar_directions) is not 0:
            if forward_dist <= 120:
                angle = 0
            for direction in ar_directions:
                print(forward_dist)
                if forward_dist <= 90:
                    if direction == Direction.LEFT:
                        counter = 0
                        cur_state = State.wall_follow_pass_left
                    elif direction == Direction.RIGHT:
                        counter = 0
                        cur_state = State.wall_follow_pass_right
                    else:
                        print(direction)
        elif 8 <= counter and forward_dist <= TURN_DISTANCE:
            cur_state = State.wall_follow_turn
        if counter >= 23 and right_dist < 30:
            cur_state = State.finish_sprint
        elif forward_dist == 0:
            cur_state = State.finish_sprint
    
    elif cur_state == State.wall_follow_turn:
        speed = 1
        top_left = rc_utils.get_lidar_average_distance(scan, 315, 10)
        top_right = rc_utils.get_lidar_average_distance(scan, 45, 10)
        angle = rc_utils.remap_range(top_right - top_left, -20, 20, -1, 1)
        if forward_dist > ACCEL_DISTANCE:
            cur_state = State.wall_follow_accel
        elif forward_dist <= 20:
            speed = -speed
    
    elif cur_state == State.wall_follow_pass_left:
        angle = -0.55
        if counter >= 1:
            cur_state = State.wall_follow_accel
    
    elif cur_state == State.wall_follow_pass_right:
        angle = 0.65
        if counter >= 1:
            cur_state = State.wall_follow_accel

    elif cur_state == State.finish_sprint:
        speed = 1
        angle = -0.02
        rc.drive.set_max_speed(0.5)

    if -0.05 < angle < 0.05:
            angle = 0
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