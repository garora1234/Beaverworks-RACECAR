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

sys.path.insert(1, "../library")
import racecar_core
import racecar_utils as rc_utils
#import copy
from enum import IntEnum

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Add any global variables here

MIN_CONTOUR_AREA = 30

CROP_FLOOR = ((360, 0), (rc.camera.get_height(), rc.camera.get_width() - 350))

CROP_FLOOR2 = ((360, 0), (rc.camera.get_height(), rc.camera.get_width()))

BLUE = ((90, 100, 100), (110, 255, 255))  # The HSV range for the color blue
GREEN = ((40, 50, 50), (80, 255, 255))  # The HSV range for the color green
RED = ((170, 50, 50), (10, 255, 255))  # The HSV range for the color red
PURPLE = ((130, 50, 50), (155, 255, 255))
ORANGE = ((7, 50, 50), (20, 255, 255))

contour_area = 0  # The area of contour
counter = 0
contour_center = None

ar_contour_center = None
ar_contour_area = 0

prev_distance = 0

speed = 0.0  # The current speed of the car
angle = 0.0  # The current angle of the car's wheels
COLOR_PRIORITY = (GREEN, RED, BLUE)

class State(IntEnum):
    DRIVING = 0
    TURNING = 1

class CONES_COLOR(IntEnum):
    BLUE = 0
    RED = 1
    NA = 2

class ConeMode(IntEnum):
    red_align = 0  
    blue_align = 1 
    red_pass = 2  
    blue_pass = 3  
    red_find = 4  
    blue_find = 5  
    red_reverse = 6  
    blue_reverse = 7  
    no_cones = 8 

cur_cone_mode = ConeMode.no_cones

BLUE_TURN = -1
RED_TURN = 1

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
CONE_MIN_CONTOUR_AREA = 100
MAX_DISTANCE = 250
REVERSE_DISTANCE = 50
STOP_REVERSE_DISTANCE = 60

CLOSE_DISTANCE = 30
FAR_DISTANCE = 120

# Add any global variables here
turn = 0
cur_state = State.DRIVING
CONES_MIN_CONTOUR_AREA = 120
CROP_AREA = 80

CONES_RED = ((170, 50, 50), (179, 255, 255))
CONES_BLUE = ((90, 120, 50), (120, 255, 255))  # The HSV range for the color blue

contour_color = CONES_COLOR.NA
last_contour_color = CONES_COLOR.NA
contour = None

ANGLE_OFFSET = 140
STOP_DISTANCE = 41

class Mode(IntEnum):
    wallFollowing = 0
    laneFollowing = 1
    coneSlaloming = 3
    lineFollowing = 4

cur_mode = Mode.wallFollowing
red_center = None
red_distance = 0
prev_red_distance = 0
blue_center = None
blue_distance = 0
prev_blue_distance = 0
line_tag_count = 0
line_detected = False
COLOR = ""
line_end = 0
tag_detected = False
image = None
depth_image = None
scan = None
ids = None
distance = 0
direction = None
########################################################################################
# Functions
########################################################################################

def resetVariables(mode = Mode.wallFollowing):

    

    global cur_mode
    global ids
    global direction
    global distance
    global contour_color
    global last_contour_color
    global contour
    global contour_area
    global counter
    global contour_center
    global image
    global prev_distance
    global speed
    global angle
    global turn
    global cur_state
    global currentIntegral
    global lastError

    global ar_contour_center
    global ar_contour_area

    global red_center
    global red_distance
    global prev_red_distance
    global blue_center
    global blue_distance
    global prev_blue_distance
    global line_tag_count
    global line_detected
    global COLOR
    global line_end
    global tag_detected

    contour_color = CONES_COLOR.NA
    last_contour_color = CONES_COLOR.NA
    contour = None

    contour_area = 0 
    counter = 0
    contour_center = None

    prev_distance = 0

    speed = 0.0  
    angle = 0.0  

    turn = 0
    cur_state = State.DRIVING

    currentIntegral = 0
    lastError = 0

    ar_contour_area = 0
    ar_contour_center = None

    red_center = None
    red_distance = 0
    prev_red_distance = 0
    blue_center = None
    blue_distance = 0
    prev_blue_distance = 0
    line_tag_count = 0
    line_detected = False
    line_end = 0
    tag_detected = False
    ids = None
    distance = 0
    direction = None

    cur_mode = mode

def update_lane_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area
    
    imageCopy = np.copy(image)
    if imageCopy is None:
        contour_center = None
        contour_area = 0
    else:
        imageCopy = rc_utils.crop(imageCopy, CROP_FLOOR[0], CROP_FLOOR[1])

        contour = None
        purpleContours = rc_utils.find_contours(imageCopy, PURPLE[0], PURPLE[1])
        orangeContours = rc_utils.find_contours(imageCopy, ORANGE[0], ORANGE[1])

        purpleContour = rc_utils.get_largest_contour(purpleContours, MIN_CONTOUR_AREA)
        orangeContour = rc_utils.get_largest_contour(orangeContours, MIN_CONTOUR_AREA)
        if purpleContour is None and orangeContour is not None:
            contour = orangeContour
        elif orangeContour is None and purpleContour is not None:
            contour = purpleContour
        elif orangeContour is not None and purpleContour is not None:
            purpleArea = rc_utils.get_contour_area(purpleContour)
            orangeArea = rc_utils.get_contour_area(orangeContour)
            if purpleArea > orangeArea:
                contour = purpleContour
            else:
                contour = orangeContour
        else:
            contour_center = None
            contour_area = 0
            return

        contour_center = rc_utils.get_contour_center(contour)
        contour_area = rc_utils.get_contour_area(contour)

        rc_utils.draw_contour(imageCopy, contour)
        rc_utils.draw_circle(imageCopy, contour_center)

      #  rc.display.show_color_image(imageCopy)


def update_line_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    global contour_center
    global contour_area

    imageCopy = np.copy(image)

    if imageCopy is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image to the floor directly in front of the car
        imageCopy = rc_utils.crop(imageCopy, CROP_FLOOR2[0], CROP_FLOOR2[1])

        # Find all of the contours of the current color
        contours = rc_utils.find_contours(imageCopy, COLOR[0], COLOR[1])

        # Select the largest contour
        contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

        if contour is not None:
            # Calculate contour information
            contour_center = rc_utils.get_contour_center(contour)
            contour_area = rc_utils.get_contour_area(contour)

            # Draw contour onto the image
            rc_utils.draw_contour(imageCopy, contour)
            rc_utils.draw_circle(imageCopy, contour_center)

        # If no contours are found for any color, set center and area accordingly
        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
     #   rc.display.show_color_image(image)


def update_cones_contour():
    global red_center
    global red_distance
    global prev_red_distance
    global blue_center
    global blue_distance
    global prev_blue_distance
    global image

    prev_red_distance = red_distance
    prev_blue_distance = blue_distance

    color_image = np.copy(image)
    depth_image = rc.camera.get_depth_image()

    if color_image is None or depth_image is None:
        red_center = None
        red_distance = 0
        blue_center = None
        blue_distance = 0
        print("No image found")
        return
    contours = rc_utils.find_contours(color_image, CONES_RED[0], CONES_RED[1])
    contour = rc_utils.get_largest_contour(contours, CONE_MIN_CONTOUR_AREA)

    if contour is not None:
        red_center = rc_utils.get_contour_center(contour)
        red_distance = rc_utils.get_pixel_average_distance(depth_image, red_center)
        if red_distance <= MAX_DISTANCE:
            rc_utils.draw_contour(color_image, contour, rc_utils.ColorBGR.green.value)
            rc_utils.draw_circle(color_image, red_center, rc_utils.ColorBGR.green.value)
        else:
            red_center = None
            red_distance = 0
    else:
        red_center = None
        red_distance = 0
    contours = rc_utils.find_contours(color_image, CONES_BLUE[0], CONES_BLUE[1])
    contour = rc_utils.get_largest_contour(contours, CONE_MIN_CONTOUR_AREA)

    if contour is not None:
        blue_center = rc_utils.get_contour_center(contour)
        blue_distance = rc_utils.get_pixel_average_distance(depth_image, blue_center)

        if blue_distance <= MAX_DISTANCE:
            rc_utils.draw_contour(color_image, contour, rc_utils.ColorBGR.yellow.value)
            rc_utils.draw_circle(
                color_image, blue_center, rc_utils.ColorBGR.yellow.value
            )
        else:
            blue_center = None
            blue_distance = 0
    else:
        blue_center = None
        blue_distance = 0

def update_ar_contour():
    """
    Finds contours in the current color image and uses them to update contour_center
    and contour_area
    """
    BLUE = ((100, 200, 50), (120, 255, 255))
    RED = ((0, 200, 50), (10, 255, 255))
    GREEN = ((50, 200, 50), (80, 255, 255))
    ORANGE = ((10, 50, 50), (20, 255, 255))
    PURPLE = ((130, 50, 50), (155, 255, 255))
    AR_COLOR_PRIORITY = [BLUE, RED, GREEN, ORANGE, PURPLE]
    global ar_contour_center
    global ar_contour_area
    global COLOR
    imageCopy = np.copy(image)

    if imageCopy is None:
        contour_center = None
        contour_area = 0
    else:
        # Search for each color in priority order
        for color in AR_COLOR_PRIORITY:
            # Find all of the contours of the current color
            contours = rc_utils.find_contours(imageCopy, color[0], color[1])

            # Select the largest contour
            contour = rc_utils.get_largest_contour(contours, MIN_CONTOUR_AREA)

            if contour is not None:
                COLOR = color
                # Calculate contour information
                contour_center = rc_utils.get_contour_center(contour)
                contour_area = rc_utils.get_contour_area(contour)

                # Draw contour onto the image
                rc_utils.draw_contour(imageCopy, contour)
                rc_utils.draw_circle(imageCopy, contour_center)

                break

        # If no contours are found for any color, set center and area accordingly
        else:
            contour_center = None
            contour_area = 0

        # Display the image to the screen
    #    rc.display.show_color_image(image)


def start():
    """
    This function is run once every time the start button is pressed
    """
    global prev_distance
    global counter
    # Have the car begin at a stop
    rc.drive.stop()

    scan = rc.lidar.get_samples()
    prev_distance = scan[0]

    # Have the car begin at a stop, in no_cones mode
    rc.drive.stop()
    counter = 0
    # Print start message

    resetVariables()
    print(">> Lab 5B - LIDAR Wall Following")


def follow_art_wall():
    global prev_distance
    global speed
    global angle
    global depth_image
    global scan
    global ids 
    global distance
    global direction

    LOOKING_ANGLE_DEGREES = 9
    LEFT_ANGLE = 321
    RIGHT_ANGLE = 39

    depth_image_copy = np.copy(depth_image)
    center_distance = rc_utils.get_pixel_average_distance(depth_image_copy,(rc.camera.get_height() // 2 + 10, rc.camera.get_width() // 2))   

    speed = rc_utils.clamp((7 - center_distance) * -0.0031, -1,1)
    distance1 = rc_utils.get_lidar_average_distance(scan, LEFT_ANGLE, LOOKING_ANGLE_DEGREES)
    distance2 = rc_utils.get_lidar_average_distance(scan, RIGHT_ANGLE, LOOKING_ANGLE_DEGREES)
    forward_dist = rc_utils.get_lidar_average_distance(scan, 90, LOOKING_ANGLE_DEGREES)
    dist_diff = distance2 - distance1
    max_speed = 0.61
    prev_distance = scan[0]
    rc.drive.set_max_speed(max_speed)
    angle = rc_utils.clamp(dist_diff * 0.0085, -1, 1)
    if distance1 > 190 and distance2 > 190:
        angle = -1
    elif distance1 > 700 and distance2 < 50:
        angle = -0.01
    elif distance1 < 50 and distance2 > 700:
        angle = 0.01

    if ids is not None and ids[0][0] == 199 and distance < 90:
        if direction == rc_utils.Direction.LEFT:
            angle = -1
        else:
            angle = 1



def cone_slalom():
    global cur_cone_mode
    global counter
    global angle
    global speed

    rc.drive.set_max_speed(0.25)

    # Align ourselves to smoothly approach and pass the red cone while it is in view
    if cur_cone_mode == ConeMode.red_align:
        # Once the red cone is out of view, enter Mode.red_pass
        if (
            red_center is None
            or red_distance == 0
            or red_distance - prev_red_distance > CLOSE_DISTANCE
        ):
            if 0 < prev_red_distance < FAR_DISTANCE:
                counter = max(1, counter)
                cur_cone_mode = ConeMode.red_pass
            else:
                cur_cone_mode = ConeMode.no_cones

        # If it seems like we are not going to make the turn, enter Mode.red_reverse
        elif (
            red_distance < REVERSE_DISTANCE
            and red_center[1] > rc.camera.get_width() // 4
        ):
            counter = REVERSE_BRAKE_TIME
            cur_cone_mode = ConeMode.red_reverse

        # Align with the cone so that it gets closer to the left side of the screen
        # as we get closer to it, and slow down as we approach
        else:
            goal_point = rc_utils.remap_range(
                red_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                0,
                rc.camera.get_width() // 4,
                True,
            )

            angle = rc_utils.remap_range(
                red_center[1], goal_point, rc.camera.get_width() // 2, 0, 1
            )
            angle = rc_utils.clamp(angle, -1, 1)

            speed = rc_utils.remap_range(
                red_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                MIN_ALIGN_SPEED,
                MAX_ALIGN_SPEED,
                True,
            )

    elif cur_cone_mode == ConeMode.blue_align:
        if (
            blue_center is None
            or blue_distance == 0
            or blue_distance - prev_blue_distance > CLOSE_DISTANCE
        ):
            if 0 < prev_blue_distance < FAR_DISTANCE:
                counter = max(1, counter)
                cur_cone_mode = ConeMode.blue_pass
            else:
                cur_cone_mode = ConeMode.no_cones
        elif (
            blue_distance < REVERSE_DISTANCE
            and blue_center[1] < rc.camera.get_width() * 3 // 4
        ):
            counter = REVERSE_BRAKE_TIME
            cur_cone_mode = ConeMode.blue_reverse
        else:
            goal_point = rc_utils.remap_range(
                blue_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                rc.camera.get_width(),
                rc.camera.get_width() * 3 // 4,
                True,
            )

            angle = rc_utils.remap_range(
                blue_center[1], goal_point, rc.camera.get_width() // 2, 0, -1
            )
            angle = rc_utils.clamp(angle, -1, 1)

            speed = rc_utils.remap_range(
                blue_distance,
                CLOSE_DISTANCE,
                FAR_DISTANCE,
                MIN_ALIGN_SPEED,
                MAX_ALIGN_SPEED,
                True,
            )

    # Curve around the cone at a fixed speed for a fixed time to pass it
    if cur_cone_mode == ConeMode.red_pass:
        angle = rc_utils.remap_range(counter, 1, 0, 0, -0.5)
        speed = PASS_SPEED
        counter -= rc.get_delta_time()

        # After the counter expires, enter Mode.blue_align if we see the blue cone,
        # and Mode.blue_find if we do not
        if counter <= 0:
            cur_cone_mode = ConeMode.blue_align if blue_distance > 0 else ConeMode.blue_find

    elif cur_cone_mode == ConeMode.blue_pass:
        angle = rc_utils.remap_range(counter, 1, 0, 0, 0.5)
        speed = PASS_SPEED

        counter -= rc.get_delta_time()
        if counter <= 0:
            cur_cone_mode = ConeMode.red_align if red_distance > 0 else ConeMode.red_find

    # If we know we are supposed to be aligning with a red cone but do not see one,
    # turn to the right until we find it
    elif cur_cone_mode == ConeMode.red_find:
        angle = 1
        speed = FIND_SPEED
        if red_distance > 0:
            cur_cone_mode = ConeMode.red_align

    elif cur_cone_mode == ConeMode.blue_find:
        angle = -1
        speed = FIND_SPEED
        if blue_distance > 0:
            cur_cone_mode = ConeMode.blue_align

    # If we are not going to make the turn, reverse while keeping the cone in view
    elif cur_cone_mode == ConeMode.red_reverse:
        if counter >= 0:
            counter -= rc.get_delta_time()
            speed = -1
            angle = 1
        else:
            angle = -1
            speed = REVERSE_SPEED
            if (
                red_distance > STOP_REVERSE_DISTANCE
                or red_center[1] < rc.camera.get_width() // 10
            ):
                counter = LONG_PASS_TIME
                cur_cone_mode = ConeMode.red_align

    elif cur_cone_mode == ConeMode.blue_reverse:
        if counter >= 0:
            counter -= rc.get_delta_time()
            speed = -1
            angle = 1
        else:
            angle = 1
            speed = REVERSE_SPEED
            if (
                blue_distance > STOP_REVERSE_DISTANCE
                or blue_center[1] > rc.camera.get_width() * 9 / 10
            ):
                counter = LONG_PASS_TIME
                cur_cone_mode = ConeMode.blue_align

    # If no cones are seen, drive forward until we see either a red or blue cone
    elif cur_cone_mode == ConeMode.no_cones:
        angle = 0
        speed = NO_CONES_SPEED

        if red_distance > 0 and blue_distance == 0:
            cur_cone_mode = ConeMode.red_align
        elif blue_distance > 0 and red_distance == 0:
            cur_cone_mode = ConeMode.blue_align
        elif blue_distance > 0 and red_distance > 0:
            cur_cone_mode = (
                ConeMode.red_align if red_distance < blue_distance else ConeMode.blue_align
            )
    
def follow_lane():
    global speed
    global angle
    rc.drive.set_max_speed(0.35)
    if not contour_center is None:
        angle = rc_utils.remap_range(contour_center[1] + 25, 0, rc.camera.get_width() - 350, -1, 1)
        speed = 1


def follow_line():
    rc.drive.set_max_speed(0.25)

    global speed
    global angle
    global line_detected
    if not contour_center is None:
        angle = rc_utils.remap_range(contour_center[1], 0, rc.camera.get_width(), -1, 1)
        speed = 1
        line_detected = True
    elif contour_center is None and line_detected == False:
        speed = 1
        angle = 0


def get_tags():
    global ids

    ar_image = np.copy(image)
    depth_image_copy = np.copy(depth_image)
    direction = None
    distance = None
    corners, ids = rc_utils.get_ar_markers(ar_image)
    if ids is not None:
        corner = corners[0]
        direction = rc_utils.get_ar_direction(corner)
        corner = corner[0]
        target = corner[0]
        if direction == rc_utils.Direction.RIGHT:
            target = corner[2]
        distance = rc_utils.get_pixel_average_distance(depth_image_copy, (int(target[1]), int(target[0])))
    return ids, direction, distance


def update():
    global cur_mode
    global line_tag_count
    global COLOR_PRIORITY
    global line_end
    global angle
    global speed
    global image
    global depth_image
    global scan
    global ids
    global direction
    global distance

    image = rc.camera.get_color_image()
    depth_image = rc.camera.get_depth_image()
    scan = rc.lidar.get_samples()

    ids, direction, distance = get_tags()
   
    if cur_mode == Mode.wallFollowing:
        follow_art_wall()
    elif cur_mode == Mode.laneFollowing:
        update_lane_contour()
        follow_lane()
    elif cur_mode == Mode.coneSlaloming:
        update_cones_contour()
        cone_slalom()
    elif cur_mode == Mode.lineFollowing:
        update_line_contour()
        follow_line()
        if line_detected == True and contour_center is None:
            line_end += 1
        elif contour_center is not None:
            line_end = 0
        if line_end == 30:
            cur_mode = Mode.wallFollowing

    if ids is not None:
        prev_mode = cur_mode
        if[1] in ids and distance < 100:
            cur_mode = Mode.laneFollowing
        elif [2] in ids:
            speed = rc_utils.remap_range(distance, 150, 800, 0.5, 1)
            if distance < 150:
                ids = None
                cur_mode = Mode.coneSlaloming
        elif [0] in ids and distance < 180:
            update_ar_contour()
            cur_mode = Mode.lineFollowing
        elif [3] in ids and distance < 100:
            cur_mode = Mode.wallFollowing
        if not (prev_mode == cur_mode):
            resetVariables(mode = cur_mode)

    speed = rc_utils.clamp(speed, -1, 1)
    angle = rc_utils.clamp(angle, -1, 1)
    rc.drive.set_speed_angle(speed, angle)

    #print("Mode: ", cur_mode, "Distance: ", distance)

    # first 1, second 3 UP , third 2, fourth 0, 5th 3, 6th 199



########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
