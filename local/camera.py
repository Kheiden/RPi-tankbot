import RPi.GPIO as GPIO
from datetime import datetime

import numpy as np
import time
import cv2
import io

from PIL import Image

class Camera():

    def __init__(self):
        self.servo_axis_x_pin = 11
        self.servo_axis_y_pin = 13

        x_axis_degrees = 0
        y_axis_degrees = 0
        # x_axis
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11,GPIO.OUT)
        self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
        self.pwm_x.start(1/18*(x_axis_degrees+90)+2)

        # y_axis
        GPIO.setup(13,GPIO.OUT)
        self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
        self.pwm_y.start(1/18*(y_axis_degrees+90)+2)

        time.sleep(1)
        self.pwm_x.stop()
        self.pwm_y.stop()
        return

    def move_camera(self, x_axis_degrees=None, y_axis_degrees=None):
        """
        # input in degrees, output in DutyCycle
        # pwm_x 0 is neutral, 90 is to the right, -90 is to the left
        # pwm_y 0 is neutral, 90 is up, -90 is down
        """
        if x_axis_degrees != None:
            self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
            self.pwm_x.start(1/18*((x_axis_degrees*-1)+90)+2)
        if y_axis_degrees != None:
            self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
            self.pwm_y.start(1/18*((y_axis_degrees*-1)+90)+2)
        """GPIO movement is not thread-blocking, so we must sleep thread"""
        time.sleep(0.75)
        self.stop_servos()

    def move_camera_smooth(self, x_start, y_start, x_end, y_end, speed):
        """
        First move x_axis, then move y_axis
        TODO: Refactor this to move both axises at the same time.
        """
        speed_dict = {
            "SLOW": 0.1,
            "FAST": 0.01
        }
        timesleep = speed_dict[speed]

        for i in range(x_start, x_end, 2):
            self.move_camera(i, None)
            time.sleep(timesleep)
            self.stop_servos()
            time.sleep(timesleep)

        for i in range(y_start, y_end, 2):
            self.move_camera(None, i)
            time.sleep(timesleep)
            self.stop_servos()
            time.sleep(timesleep)

        self.stop_servos()

    def stop_servos(self):
        self.pwm_x.stop()
        self.pwm_y.stop()

    def take_stereo_photo(self, x_res, y_res, type="combined"):
        """
        type="combined" (or any other value) is a single .JPG file
        type="separate" is two separate .JPG files
        """
        CAMERA_WIDTH = x_res
        CAMERA_HEIGHT = y_res
        print("CAMERA_WIDTH: {}, CAMERA_HEIGHT:{}".format(CAMERA_WIDTH, CAMERA_HEIGHT))

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))



        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))


        for i in range(30):
            #This is used to "warm up" the camera before retrieving the photo
            left.grab()
            right.grab()
        _, rightFrame = right.retrieve()
        _, leftFrame = left.retrieve()
        right.release()
        left.release()


        imgRGB_right=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2RGB)
        imgRGB_left=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2RGB)
        if type == "separate":
            jpg_image_right = Image.fromarray(imgRGB_right)
            jpg_image_left = Image.fromarray(imgRGB_left)
            filename = datetime.now().strftime("%F_%H-%M-%S.%f")
            jpg_image_right.save("/home/pi/RPi-tankbot/local/frames/{}_right.jpg".format(filename), format='JPEG')
            jpg_image_left.save("/home/pi/RPi-tankbot/local/frames/{}_left.jpg".format(filename), format='JPEG')

            width_right, height_right = jpg_image_right.size
            width_left, height_left = jpg_image_left.size
            if ((width_right == width_left) and (height_right == height_left)):
                return width_right, height_right
            else:
                # This shouldn't happen.  If it does, error out.
                return 0, 0
        else:
            # if not defined, then it's combined ;)
            imgRGB_combined = np.concatenate((imgRGB_left, imgRGB_right), axis=1)
            jpg_image = Image.fromarray(imgRGB_combined)
            filename = datetime.now().strftime("%F_%H-%M-%S.%f")
            jpg_image.save("/home/pi/RPi-tankbot/local/frames/{}_combined.jpg".format(filename), format='JPEG')

            width, height = jpg_image.size
            return width, height

    def start_right_camera(self):
        CAMERA_WIDTH = 640
        CAMERA_HEIGHT = 480

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        right.set(cv2.CAP_PROP_FPS,30)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        while True:
            right.grab()
            _, rightFrame = right.retrieve()
            imgRGB=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2RGB)
            jpg_image = Image.fromarray(imgRGB)
            bytes_array = io.BytesIO()
            jpg_image.save(bytes_array, format='JPEG')
            jpg_image_bytes = bytes_array.getvalue()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_image_bytes + b'\r\n')
        right.release()

    def start_left_camera(self):
        CAMERA_WIDTH = 640
        CAMERA_HEIGHT = 480

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        left.set(cv2.CAP_PROP_FPS,30)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        while True:
            left.grab()
            _, leftFrame = left.retrieve()
            imgRGB=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2RGB)
            jpg_image = Image.fromarray(imgRGB)
            bytes_array = io.BytesIO()
            jpg_image.save(bytes_array, format='JPEG')
            jpg_image_bytes = bytes_array.getvalue()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_image_bytes + b'\r\n')
        left.release()

    def start_left_and_right_cameras(self):

        CAMERA_WIDTH = 1280
        CAMERA_HEIGHT = 720

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        left.set(cv2.CAP_PROP_FPS,1)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        right.set(cv2.CAP_PROP_FPS,1)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        while(True):
            if not (left.grab() and right.grab()):
                print("No more frames")
                break

            _, leftFrame = left.retrieve()
            _, rightFrame = right.retrieve()

            imgRGB_right=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2RGB)
            imgRGB_left=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2RGB)
            imgRGB_combined = np.concatenate((imgRGB_left, imgRGB_right), axis=1)
            jpg_image = Image.fromarray(imgRGB_combined)

            bytes_array = io.BytesIO()
            jpg_image.save(bytes_array, format='JPEG')
            jpg_image_bytes = bytes_array.getvalue()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpg_image_bytes + b'\r\n')


        left.release()
        right.release()
        cv2.destroyAllWindows()
