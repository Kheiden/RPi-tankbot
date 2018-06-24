import RPi.GPIO as GPIO
from datetime import datetime

from diskcache import Cache
import numpy as np
import random
import glob
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


    def calibrate_camera(self, cam_num=0, save_chessboard=False):
        """
        cam_num 0 is left and 1 is right.

        Code sample based on:
        http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
        and
        https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
        """
        # Arrays to store object points and image points from all the images.
        right_or_left = ["_right" if cam_num==1 else "_left"][0]

        CHECKERBOARD = (6,9)

        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

        objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

        _img_shape = None

        cache = Cache('/home/pi/calibration_data/calibrationcachedata{}'.format(right_or_left))
        images = glob.glob('/home/pi/calibration_frames/*{}.jpg'.format(right_or_left))

        if 'objpoints' and 'imgpoints' in cache:
            print("Camera calibration data has been found in cache.")
            objpoints = cache['objpoints']
            imgpoints = cache['imgpoints']
        else:
            print("Camera calibration data not found in cache.")

            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.
            num_chessboards_found = []

            for file_name in images:
                img_uncropped = cv2.imread(file_name)
                # crop the image
                # y start at 0, end at 1080
                # x start at 240, end at 1440
                #img = img_uncropped[0:1080, 240:1440]
                img = img_uncropped

                if _img_shape == None:
                    _img_shape = img.shape[:2]
                else:
                    assert _img_shape == img.shape[:2], "All images must share the same size."

                camera_width = 1920
                crop_width = 1440

                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)
                    cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                    imgpoints.append(corners)

                    # Draw and display the corners
                    if save_chessboard == True:
                        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
                        jpg_image = Image.fromarray(img)
                        jpg_image.save(file_name.replace("calibration_frames", "chessboard_frames"), format='JPEG')
                        #jpg_image.save(r"C:\Users\kurtw\Documents\raspberry pi\Local Test Code\chessboard\output.jpg", format='JPEG')
                    num_chessboards_found.append(True)
            print("Saving calibration data to cache...")
            cache['objpoints'] = objpoints
            cache['imgpoints'] = imgpoints
            cache.close()

        # Starting from here if cache is found...

        # Opencv sample code uses the var 'grey' from the last opened picture
        # I'm going to choose one at random

        file_name = random.sample(images, 1)[0]
        img = cv2.imread(file_name)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        N_OK = len(objpoints)
        DIM= (1920, 1080)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        rms, _, _, _, _ = \
            cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                gray.shape[::-1],
                K,
                D,
                rvecs,
                tvecs,
                calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )
        #print("Found " + str(N_OK) + " valid images for calibration")
        #print("DIM=" + str(_img_shape[::-1]))
        #print("K=np.array(" + str(K.tolist()) + ")")
        #rint("D=np.array(" + str(D.tolist()) + ")")

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        np.savez('/home/pi/calibration_data/camera_calibration{}.npz'.format(right_or_left), mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

        img_uncropped = cv2.imread('/home/pi/input_output/input{}.jpg'.format(right_or_left))
        #img = img_uncropped[0:1080, 240:1440]
        img = img_uncropped
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        # undistort

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        cv2.imwrite('/home/pi/input_output/output{}.jpg'.format(right_or_left), np.hstack((img, undistorted_img)))

        return True


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


        for i in range(45):
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
