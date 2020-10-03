import sys
import cv2 as cv
import numpy as np
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils
import cv2
import tensorflow as tf
import math
rc = racecar_core.create_racecar()
model = None
def prepare(filepath):
    IMG_SIZE = 128
    img_array = cv.imread(filepath, cv2.IMREAD_COLOR)
    img_array = img_array/255.0
    new_array = cv.resize(img_array, (IMG_SIZE, IMG_SIZE))
    return new_array.reshape(-1, IMG_SIZE, IMG_SIZE, 3)
def start():
    global model
    # Have the car begin at a stop
    rc.drive.stop()
    # Print start message
    model = tf.keras.models.load_model("RACECAR.model")
def update():
    global model
    speed = 0.5
    angle = rc.controller.get_joystick(rc.controller.Joystick.LEFT)[0]
    image = get_lidar_image()
    cv.imwrite("cur_image.jpg", image)
    CATEGORIES = ["forward", "right", "left"]
    prediction = model.predict_classes([prepare('cur_image.jpg')])
    label = CATEGORIES[int(prediction[0])]
    if label == "forward":
        angle = 0
    elif label == "left":
        angle = -1
    elif label == "right":
        angle = 1
    
    #os.makedirs("data/" + label + "/", exist_ok=True)
    #cv.imwrite("data/" + label + "/" + str(uuid.uuid4()) + ".jpg", rc.camera.get_color_image())
    rc.drive.set_speed_angle(speed, angle)
########################################################################################
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
if __name__ == "__main__":
    rc.set_start_update(start, update, None)
    rc.go()