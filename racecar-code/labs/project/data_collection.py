import sys
import cv2 as cv
import numpy as np
import math

sys.path.insert(0, "../../library")
import racecar_core
import racecar_utils as rc_utils
import uuid
import os

rc = racecar_core.create_racecar()

def get_lidar_image():
    samples = rc.lidar.get_samples()
    radius= 128
    max_range= 1000
    image = np.zeros((2 * radius, 2 * radius, 3), np.uint8, "C")
    num_samples: int = len(samples)

    for i in range(num_samples):
        if 0 < samples[i] < max_range:
            angle: float = 2 * math.pi * i / num_samples
            length: float = radius * samples[i] / max_range
            r: int = int(radius - length * math.cos(angle))
            c: int = int(radius + length * math.sin(angle))
            image[r][c][2] = 255

    # Draw a green dot to denote the car
    rc_utils.draw_circle(
        image,
        (radius, radius),
        rc_utils.ColorBGR.green.value,
        2,
    )
    return image

def start():

    # Have the car begin at a stop
    rc.drive.stop()

    # Print start message

def update():
    speed = 0.5 
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    label = "forward"
    if abs(angle - 0.1) < 0.2:
        angle = 0
    elif angle < 0:
        label = "left"
        angle = -1
    else:
        label = "right"
        angle = 1
    os.makedirs("data/" + label + "/", exist_ok=True)
    path = "data/" + label + "/" + str(uuid.uuid4()) + ".jpg"
    #print(path)
    
    img = rc.camera.get_depth_image()
    if not cv.imwrite(path, img):
       raise Exception("Could not write image")

    rc.drive.set_speed_angle(speed, angle)
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
