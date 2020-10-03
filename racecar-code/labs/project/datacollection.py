import sys
import cv2 as cv
import numpy as np

sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
import uuid
import os
import math

rc = racecar_core.create_racecar()

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
    img = get_lidar_image()
    
    if not cv.imwrite(path, img):
        raise Exception("Could not write image")

    rc.display.show_lidar
    rc.drive.set_speed_angle(speed, angle)

def get_lidar_image():
    samples = rc.lidar.get_samples()
    radius= 64
    max_range= 300
    image = np.zeros((2 * radius, 2 * radius, 3), np.uint8, "C")
    num_samples: int = len(samples)

    for i in range(num_samples):
        if 0 < samples[i] < max_range:
            angle: float = 2 * math.pi * i / num_samples
            length: float = radius * samples[i] / max_range
            r: int = int(radius - length * math.cos(angle))
            c: int = int(radius + length * math.sin(angle))
            image[r][c][2] = 255

    return image
########################################################################################


if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()
