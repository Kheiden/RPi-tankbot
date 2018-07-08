import RPi.GPIO as GPIO
from datetime import datetime

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

        self.home_dir = "/home/pi"
        return

    def create_3d_point_cloud(self, imgL, disparity_map):
        """
        Based on sample code from OpenCV
        """
        print('generating 3d point cloud...',)
        h, w = imgL.shape[:2]
        f = 0.8*w                          # guess for focal length
        Q = np.float32([[1, 0, 0, -0.5*w],
                        [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                        [0, 0, 0,     -f], # so that y-axis looks up
                        [0, 0, 1,      0]])
        points = cv2.reprojectImageTo3D(disparity_map, Q)
        colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
        mask = disparity_map > disparity_map.min()
        out_points = points[mask]
        out_colors = colors[mask]

        verts = out_points
        colors = out_colors

        fn = '/home/pi/RPi-tankbot/local/frames/out.ply'
        ply_header = '''ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar red
        property uchar green
        property uchar blue
        end_header
        '''

        verts = verts.reshape(-1, 3)
        colors = colors.reshape(-1, 3)
        verts = np.hstack([verts, colors])
        with open(fn, 'wb') as f:
            f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

        print('%s saved' % 'out.ply')
        return True


    def realtime_disparity_map_stream(self, time_on):
        frame_counter = 0
        processing_time01 = cv2.getTickCount()
        res_x = 640
        res_y = 480
        npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))
        while True:
            imgL, imgR = self.take_stereo_photo(res_x, res_y, type="image_array")
            result = self.create_disparity_map(imgL, imgR, res_x, res_y, leftMapX=npzfile['leftMapX'],
                        leftMapY=npzfile['leftMapY'], rightMapX=npzfile['rightMapX'], rightMapY=npzfile['rightMapY'],
                        save_disparity_image=False)
            frame_counter += 1
            processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
            if processing_time >= time_on:
                return processing_time, frame_counter

    def create_disparity_map(self, imgLeft, imgRight, res_x=640, res_y=480, leftMapX=None,
                leftMapY=None, rightMapX=None, rightMapY=None, save_disparity_image=False):
        """
        create_disparity_map takes in two undistorted images from left and right cameras.
        This function will undistort the images by passing each image to undistort_image

        """
        # take two photos
        file_name = "disparity_test"

        #if leftMapX isn't assigned, then assume that none are assigned.
        if leftMapX is None:
            npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

            #imageSize = tuple(npzfile['imageSize'])
            leftMapX = npzfile['leftMapX']
            leftMapY = npzfile['leftMapY']
            #leftROI = tuple(npzfile['leftROI'])
            rightMapX = npzfile['rightMapX']
            rightMapY = npzfile['rightMapY']
        #rightROI = tuple(npzfile['rightROI'])

        #imgLeft_jpg = Image.fromarray(imgLeft)
        #imgRight_jpg = Image.fromarray(imgRight)

        #imgLeft_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_distorted_left.jpg".format(file_name), format='JPEG')
        #imgRight_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_distorted_right.jpg".format(file_name), format='JPEG')


        imgLeft = cv2.remap(imgLeft, leftMapX, leftMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgRight = cv2.remap(imgRight, rightMapX, rightMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        #imgLeft_jpg = Image.fromarray(imgLeft)
        #imgRight_jpg = Image.fromarray(imgRight)

        #imgLeft_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_color_left.jpg".format(file_name), format='JPEG')
        #imgRight_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_color_right.jpg".format(file_name), format='JPEG')


        grayLeft = cv2.cvtColor(imgLeft, cv2.COLOR_RGB2GRAY)
        grayRight = cv2.cvtColor(imgRight, cv2.COLOR_RGB2GRAY)

        #imgLeft_jpg = Image.fromarray(grayLeft)
        #imgRight_jpg = Image.fromarray(grayRight)

        #imgLeft_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_gray_left.jpg".format(file_name), format='JPEG')
        #imgRight_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_gray_right.jpg".format(file_name), format='JPEG')


        # Initialize the stereo block matching object
        stereo = cv2.StereoBM_create()
        stereo.setBlockSize(9)
        stereo.setMinDisparity(0)
        stereo.setNumDisparities(48)
        stereo.setDisp12MaxDiff(1)
        stereo.setSpeckleRange(0)
        stereo.setSpeckleWindowSize(0)
        #stereo.setROI1(leftROI)
        #stereo.setROI2(rightROI)
        stereo.setPreFilterCap(63) # was 63
        stereo.setPreFilterSize(15) # was 15

        stereo.setUniquenessRatio(0)
        stereo.setTextureThreshold(0)

        # Compute the disparity image
        disparity = stereo.compute(grayLeft, grayRight)
        # Normalize the image for representation
        norm_coeff = 255 / disparity.max()
        disparity_normalized = disparity * norm_coeff / 255
        # No clue why but the above normalization changes the imgL, so I need to readjust it
        imgLeft = imgLeft / 255

        if save_disparity_image == True:
            jpg_image = Image.fromarray(disparity_normalized*255)
            jpg_image = jpg_image.convert('RGB')
            jpg_image.save("/home/pi/RPi-tankbot/local/frames/{}_disparity_map.jpg".format(file_name), format='JPEG')

        return imgLeft, disparity_normalized

    def calibrate_stereo_cameras(self, res_x=640, res_y=480):
        # We need a lot of variables to calibrate the stereo camera
        """
        Based on code from:
        https://gist.github.com/aarmea/629e59ac7b640a60340145809b1c9013
        """
        processing_time01 = cv2.getTickCount()
        objectPoints = None

        rightImagePoints = None
        rightCameraMatrix = None
        rightDistortionCoefficients = None

        leftImagePoints = None
        leftCameraMatrix = None
        leftDistortionCoefficients = None

        rotationMatrix = None
        translationVector = None

        imageSize= (res_x, res_y)
        #imageSize = (1920, 1080)

        TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
        OPTIMIZE_ALPHA = 0.25

        try:
            npz_file = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))
            processing_time02 = cv2.getTickCount()
            processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
            return processing_time
        except:
            pass

        for cam_num in [0, 1]:
            right_or_left = ["_right" if cam_num==1 else "_left"][0]

            try:
                npz_file = np.load('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir, res_y, right_or_left))

                list_of_vars = ['map1', 'map2', 'objpoints', 'imgpoints', 'camera_matrix', 'distortion_coeff']
                print(sorted(npz_file.files))

                if sorted(list_of_vars) == sorted(npz_file.files):
                    print("Camera calibration data has been found in cache.")
                    map1 = npz_file['map1']
                    map2 = npz_file['map2']
                    objectPoints = npz_file['objpoints']
                    if right_or_left == "_right":
                        rightImagePoints = npz_file['imgpoints']
                        rightCameraMatrix = npz_file['camera_matrix']
                        rightDistortionCoefficients = npz_file['distortion_coeff']
                    if right_or_left == "_left":
                        leftImagePoints = npz_file['imgpoints']
                        leftCameraMatrix = npz_file['camera_matrix']
                        leftDistortionCoefficients = npz_file['distortion_coeff']
                else:
                    print("Camera data file found but data corrupted.")
            except:
                #If the file doesn't exist
                print("Camera calibration data not found in cache.")
                return False


        print("Calibrating cameras together...")

        leftImagePoints = np.asarray(leftImagePoints, dtype=np.float64)
        rightImagePoints = np.asarray(rightImagePoints, dtype=np.float64)

        (RMS, _, _, _, _, rotationMatrix, translationVector) = cv2.fisheye.stereoCalibrate(
                objectPoints, leftImagePoints, rightImagePoints,
                leftCameraMatrix, leftDistortionCoefficients,
                rightCameraMatrix, rightDistortionCoefficients,
                imageSize, None, None,
                cv2.CALIB_FIX_INTRINSIC, TERMINATION_CRITERIA)

        print("Root Means Squared:", RMS)

        print("Rectifying cameras...")
        R1 = np.zeros([3,3])
        R2 = np.zeros([3,3])
        P1 = np.zeros([3,4])
        P2 = np.zeros([3,4])
        Q = np.zeros([4,4])

        (leftRectification, rightRectification, leftProjection, rightProjection,
                dispartityToDepthMap) = cv2.fisheye.stereoRectify(
                        leftCameraMatrix, leftDistortionCoefficients,
                        rightCameraMatrix, rightDistortionCoefficients,
                        imageSize, rotationMatrix, translationVector,
                        0, R2, P1, P2, Q,
                        cv2.CALIB_ZERO_DISPARITY, (0,0) , 0, 0)

        print("Saving calibration...")
        leftMapX, leftMapY = cv2.fisheye.initUndistortRectifyMap(
                leftCameraMatrix, leftDistortionCoefficients, leftRectification,
                leftProjection, imageSize, cv2.CV_16SC2)
        rightMapX, rightMapY = cv2.fisheye.initUndistortRectifyMap(
                rightCameraMatrix, rightDistortionCoefficients, rightRectification,
                rightProjection, imageSize, cv2.CV_16SC2)

        np.savez_compressed('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y), imageSize=imageSize,
                leftMapX=leftMapX, leftMapY=leftMapY,
                rightMapX=rightMapX, rightMapY=rightMapY)

        processing_time02 = cv2.getTickCount()
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        return processing_time


    def calibrate_camera(self, cam_num=0, res_x=640, res_y=480):
        """
        cam_num 0 is left and 1 is right.

        Code sample based on:
        http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
        and
        https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
        """
        # Arrays to store object points and image points from all the images.
        processing_time01 = cv2.getTickCount()
        right_or_left = ["_right" if cam_num==1 else "_left"][0]
        CHECKERBOARD = (6,9)

        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

        objp = np.zeros( (CHECKERBOARD[0]*CHECKERBOARD[1], 1, 3) , np.float64)
        objp[:,0, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

        _img_shape = None
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        """
        TODO:
        if the file exists, then load it
        If the file doesn't exist, then calibrate the cameras and save result to file
        """

        images = glob.glob('{}/calibration_frames/{}p/*{}.jpg'.format(self.home_dir, res_y, right_or_left))
        calbrate_cameras = None

        try:
            npz_file = np.load('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir, res_y, right_or_left))
            if 'map1' and 'map2' in npz_file.files:
                print("Camera calibration data has been found in cache.")
                map1 = npz_file['map1']
                map2 = npz_file['map2']
            else:
                print("Camera data file found but data corrupted.")
                calbrate_cameras = True
        except:
            # If the file doesn't exist
            print("Camera calibration data not found in cache.")
            calbrate_cameras = True

        if calbrate_cameras == True:
            print("Calibrating cameras...")
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            for index, file_name in enumerate(images):
                print(index, file_name)
                img = cv2.imread(file_name)
                if _img_shape == None:
                    _img_shape = img.shape[:2]
                else:
                    assert _img_shape == img.shape[:2], "All images must share the same size."

                gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)
                    cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                    imgpoints.append(corners)
                else:
                    print("Error! couldn't find chessboard corner in below file!!")
                    print(file_name)

            # Opencv sample code uses the var 'grey' from the last opened picture
            N_OK = len(objpoints)
            DIM= (res_x, res_y)
            K = np.zeros((3, 3))
            D = np.zeros((4, 1))
            rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
            tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

            rms, camera_matrix, distortion_coeff, _, _ = \
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

            print("Root Means Squared:", rms)

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
            np.savez('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir,  res_y, right_or_left),
                map1=map1, map2=map2, objpoints=objpoints, imgpoints=imgpoints,
                camera_matrix=camera_matrix, distortion_coeff=distortion_coeff)


        # Starting from here if cache is found...
        processing_time02 = cv2.getTickCount()
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        return processing_time

    def undistort_image(self, img, cam_num):
        """
        # Takes an image in as a numpy array and undistorts it
        """
        processing_time01 = cv2.getTickCount()
        right_or_left = ["_right" if cam_num==1 else "_left"][0]

        h, w = img.shape[:2]
        print("Undistorting picture with (width, height):", (w, h))
        try:
            npz_file = np.load('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir, h, right_or_left))
            if 'map1' and 'map2' in npz_file.files:
                #print("Camera calibration data has been found in cache.")
                map1 = npz_file['map1']
                map2 = npz_file['map2']
            else:
                print("Camera data file found but data corrupted.")
                return False
        except:
            print("Camera calibration data not found in cache.")
            return False

        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        processing_time02 = cv2.getTickCount()
        #cv2.imwrite('{}/input_output/output{}.jpg'.format(self.home_dir, right_or_left), np.hstack((img, undistorted_img)))
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        #Return an image the same size as the input image
        return (processing_time, undistorted_img[0:h, 0:w])

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

    def take_stereo_photo(self, res_x, res_y, filename=None, type="combined"):
        """
        type="combined" (or any other value) is a single .JPG file
        type="separate" is two separate .JPG files
        """
        #processing_time01 = cv2.getTickCount()
        CAMERA_HEIGHT = res_y
        CAMERA_WIDTH = res_x

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
            if filename == None:
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
        elif type == "together":
            imgRGB_combined = np.concatenate((imgRGB_left, imgRGB_right), axis=1)
            jpg_image = Image.fromarray(imgRGB_combined)
            if filename == None:
                filename = datetime.now().strftime("%F_%H-%M-%S.%f")
            jpg_image.save("/home/pi/RPi-tankbot/local/frames/{}_combined.jpg".format(filename), format='JPEG')

            width, height = jpg_image.size
            return width, height
        elif type == "image_array":
            #processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
            return imgRGB_left, imgRGB_right
        else:
            return

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
