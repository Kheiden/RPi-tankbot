import RPi.GPIO as GPIO
from datetime import datetime
from PIL import Image
from tankbot_logs import RobotLog

import numpy as np
import threading
import movement
import random
import queue
import glob
import time
import cv2
import io


class Camera():

    def __init__(self):
        """
        This class is used for all of the functions related to the camera except:
        - moving the camera servos
        - calibrating the cameras
        """
        self.log = RobotLog()
        self.m = movement.Movement()
        self.home_dir = "/home/pi"


    def create_3d_point_cloud(self, imgL, disparity_map, file_num):
        """
        Based on sample code from OpenCV
        """
        self.log.debug('generating 3d point cloud...')
        h, w = imgL.shape[:2]
        f = 0.8*w                          # guess for focal length
        Q = np.float32([[1, 0, 0, -0.5*w],
                        [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                        [0, 0, 0,     -f], # so that y-axis looks up
                        [0, 0, 1,      0]])
        points = cv2.reprojectImageTo3D(disparity_map, Q)
        #cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
        colors = imgL
        mask = disparity_map > disparity_map.min()
        out_points = points[mask]
        out_colors = colors[mask]

        verts = out_points
        colors = out_colors

        file_name = "{}/RPi-tankbot/local/cloud_points/out{}.ply".format(self.home_dir, file_num)
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
        with open(file_name, 'wb') as f:
            f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

        self.log.debug("3d cloud point saved to:", file_name)
        return True


    def realtime_disparity_map_stream(self, time_on, action=None,
        save_disparity_image=False,
        override_warmup=False,
        autonomous_routine=None):

        frame_counter = 0

        res_x = 640
        res_y = 480
        npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        processing_time01 = cv2.getTickCount()
        while True:
            disparity_map_time = cv2.getTickCount()
            # Stop moving
            self.m.stop()
            imgL, imgR = self.take_stereo_photo(
                res_x,
                res_y,
                right=right,
                left=left,
                type="image_array",
                override_warmup=override_warmup)
            result = self.create_disparity_map(imgL, imgR, res_x, res_y, npzfile=npzfile, save_disparity_image=save_disparity_image)
            if action is not None:
                threshold = action[0]
                num_threshold = action[1]
                b = np.where(result[1] > threshold)
                num_pixels_above_threshold = len(b[0])
                self.log.debug("Threshold: {}/255, num_threshold: {}, num_pixels_above_threshold: {}".format(
                    threshold,
                    num_threshold,
                    num_pixels_above_threshold
                ))
                if num_pixels_above_threshold >= num_threshold:
                    # This means that we need to stop the robot ASAP
                    self.log.debug("Object detected too close!")
                    if action[2] == 'stop_if_close':
                        self.m.stop()
                        self.log.debug("Stopping robot to avoid collision")
                        return None, None, action[2]

                    if action[2] == 'rotate_random':
                        direction = random.choice(["right", "left"])
                        self.log.debug("Rotating {} to avoid obstacle".format(direction))
                        #move left or right
                        self.m.rotate_on_carpet(direction=direction,
                            movement_time=6,
                            sleep_speed=0.25)
                else:
                    # this means that there are no objects in the way
                    disparity_map_time = (cv2.getTickCount() - disparity_map_time)/ cv2.getTickFrequency()
                    self.log.debug("Disparity map took {} seconds to process".format(disparity_map_time))
                    # use the threadblocking forward command with the sleep parameter set
                    self.m.forward(1)

                frame_counter += 1
                processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
                if processing_time >= time_on:
                    return processing_time, frame_counter, None

    def create_disparity_map(self, imgLeft, imgRight, res_x=640, res_y=480, npzfile=None, save_disparity_image=False):
        """
        create_disparity_map takes in two undistorted images from left and right cameras.
        This function will undistort the images by passing each image to undistort_image

        param:
          imgLeft (BGR image only)
          imgRight (BGR image only)
          res_x: width of the picture to be taken
          res_y: height of the picture to be taken
          npzfile: location to the stereo calibration data
          save_disparity_image: (bool) Whether or not to save the image as a normalized jpg
        """
        #file_name = "disparity_test"

        #imgLeft, imgRight = self.take_stereo_photo(res_x, res_y, file_name, "image_array")
        if npzfile is None:
            npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

        imageSize = tuple(npzfile['imageSize'])
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



        self.log.debug("Successful 2")
        imgLeft = cv2.remap(imgLeft, leftMapX, leftMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgRight = cv2.remap(imgRight, rightMapX, rightMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        #if save_disparity_image == True:
        #    imgLeft_jpg = Image.fromarray(imgLeft)
        #    imgRight_jpg = Image.fromarray(imgRight)

        #    imgLeft_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_color_left.jpg".format(file_name), format='JPEG')
        #    imgRight_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_color_right.jpg".format(file_name), format='JPEG')

        grayLeft = cv2.cvtColor(imgLeft,cv2.COLOR_BGR2GRAY)
        grayRight = cv2.cvtColor(imgRight,cv2.COLOR_BGR2GRAY)

        #if save_disparity_image == True:
        #    imgLeft_jpg = Image.fromarray(grayLeft)
        #    imgRight_jpg = Image.fromarray(grayRight)

        #    imgLeft_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_gray_left.jpg".format(file_name), format='JPEG')
        #    imgRight_jpg.save("/home/pi/RPi-tankbot/local/frames/{}_gray_right.jpg".format(file_name), format='JPEG')

        # Initialize the stereo block matching object
        stereo = cv2.StereoBM_create()
        stereo.setBlockSize(25) # was 9
        stereo.setMinDisparity(0)
        stereo.setNumDisparities(48)
        stereo.setDisp12MaxDiff(2)
        stereo.setSpeckleRange(3) # was 0
        stereo.setSpeckleWindowSize(65) # was 0
        #stereo.setROI1(leftROI)
        #stereo.setROI2(rightROI)
        stereo.setPreFilterCap(10) # was 63
        stereo.setPreFilterSize(5) # was 5

        stereo.setUniquenessRatio(4) # was 3
        stereo.setTextureThreshold(0)

        self.log.debug("Successful 3")
        # Compute the disparity image
        disparity = stereo.compute(grayLeft, grayRight)
        self.log.debug("Successful 4")
        # Normalize the image for representation
        norm_coeff = 255 / disparity.max()
        disparity_normalized = disparity * norm_coeff / 255
        # No clue why but the above normalization changes the imgL, so I need to readjust it
        #imgLeft = imgLeft / 255
        self.log.debug("Successful 5")
        if save_disparity_image == True:
            jpg_image = Image.fromarray(disparity_normalized*255)
            jpg_image = jpg_image.convert('RGB')
            timestamp = time.time()
            jpg_image.save("/home/pi/RPi-tankbot/local/frames/disparity_map_{}.jpg".format(timestamp), format='JPEG')
            jpg_image = Image.fromarray(imgLeft)
            jpg_image.save("/home/pi/RPi-tankbot/local/frames/disparity_map_{}_color.jpg".format(timestamp), format='JPEG')
            self.log.debug("Successful 6")

        return imgLeft, disparity


    def undistort_image(self, img, cam_num):
        """
        # Takes an image in as a numpy array and undistorts it
        """
        self.log.debug("Starting Timer...")
        processing_time01 = cv2.getTickCount()
        right_or_left = ["_right" if cam_num==1 else "_left"][0]
        self.log.debug("Determining height and width of image...")
        h, w = img.shape[:2]
        self.log.debug("Undistorting picture with (width, height): {},{}".format(w, h))
        try:
            npz_file = np.load('{}/calibration_data/{}p/camera_calibration{}.npz'.format(self.home_dir, h, right_or_left))
            if 'map1' and 'map2' in npz_file.files:
                #self.log.debug("Camera calibration data has been found in cache.")
                map1 = npz_file['map1']
                map2 = npz_file['map2']
            else:
                self.log.debug("Camera data file found but data corrupted.")
                return False
        except:
            self.log.debug("Camera calibration data not found in cache.")
            return False

        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        processing_time02 = cv2.getTickCount()
        #cv2.imwrite('{}/input_output/output{}.jpg'.format(self.home_dir, right_or_left), np.hstack((img, undistorted_img)))
        processing_time = (processing_time02 - processing_time01)/ cv2.getTickFrequency()
        #Return an image the same size as the input image
        return (processing_time, undistorted_img[0:h, 0:w])


    def take_stereo_photo(self, res_x, res_y, right, left, override_warmup, filename=None, type="combined", quick_capture=False):
        """
        type="combined" (or any other value) is a single .JPG file
        type="separate" is two separate .JPG files

        quick_capture is used to take the photos as fast as possible.
            This returns greyscale photos to be used in the disparity_map_stream
            along with other speed improvements (might combine with left/right)
        """
        #processing_time01 = cv2.getTickCount()
        CAMERA_HEIGHT = res_y
        CAMERA_WIDTH = res_x

        #self.log.debug("Photo Resolution: res_y: {}, res_x: {}".format(res_y, res_x))

        if override_warmup or quick_capture == True:
            num_photos = 1
        else:
            num_photos = 45

        for i in range(num_photos):
            #This is used to "warm up" the camera before retrieving the photo
            ret_left = left.grab()
            ret_right = right.grab()
        _, rightFrame = right.retrieve()
        _, leftFrame = left.retrieve()

        #self.log.debug(ret_left, ret_right)

        if ret_left == False:
            self.log.debug("Left Camera Not Working")
            return (None, None)
        if ret_right == False:
            self.log.debug("Right Camera Not Working")
            return (None, None)

        if quick_capture == False:
            imgRGB_right=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2RGB)
            imgRGB_left=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2RGB)
        elif quick_capture == True:
            #imgGRAY_right=cv2.cvtColor(rightFrame,cv2.COLOR_BGR2GRAY)
            #imgGRAY_left=cv2.cvtColor(leftFrame,cv2.COLOR_BGR2GRAY)
            #return imgGRAY_left, imgGRAY_right
            return leftFrame, rightFrame
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
        elif type == "combined":
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
            self.log.debug("Incorrect Parameter set for `type`")
            raise AttributeError


    def threaded_disparity_map(self, npzfile):
        res_x = 640
        res_y = 480
        #time.sleep(1)
        while True:
            self.log.debug("self.input_queue.qsize(): {}".format(self.input_queue.qsize()))
            imgBGR_left, imgBGR_right = self.input_queue.get(timeout=8)

            result = self.create_disparity_map(imgBGR_left, imgBGR_right, res_x, res_y, npzfile=npzfile, save_disparity_image=False)
            disparity = result[1]

            norm_coeff = 255 / disparity.max()
            disparity_normalized = disparity * norm_coeff / 255

            jpg_image = Image.fromarray(disparity_normalized*255)
            jpg_image = jpg_image.convert('RGB')

            bytes_array = io.BytesIO()
            jpg_image.save(bytes_array, format='JPEG')
            jpg_image_bytes = bytes_array.getvalue()
            self.log.debug("~~~putting frame in output queue!")
            self.output_queue.put(jpg_image_bytes)

            if self.input_queue.empty():
                break


    def threaded_take_stereo_photo(self):
        res_x = 640
        res_y = 480
        max_queue_size = 900 # 30 seconds at 30 fps

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        while self.input_queue.qsize() < max_queue_size:
            #imgL, imgR = self.take_stereo_photo(res_x, res_y, type="image_array", override_warmup=True)
            imgBGR_left, imgBGR_right = self.take_stereo_photo(res_x, res_y,
                right=right,
                left=left,
                type="image_array",
                override_warmup=True,
                quick_capture=True)
            self.log.debug("Putting image in self.input_queue")

            # TODO- remove this sleep command
            time.sleep(0.5 + (self.input_queue.qsize()/100))

            self.input_queue.put((imgBGR_left, imgBGR_right))

    def start_disparity_map(self):
        res_x = 640
        res_y = 480
        npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

        self.input_queue = queue.Queue()
        self.output_queue = queue.Queue()

        # first, start the thread for the camera
        thread = threading.Thread(group=None, target=self.threaded_take_stereo_photo, name="Thread_camera")
        self.log.debug("Starting Thread_camera")
        thread.start()
        self.log.debug("Sleeping main thread...")
        time.sleep(0.5)
        self.log.debug("main thread waking up")
        # second, start the threads for disparity_map processing
        for i in range(3):
            thread = threading.Thread(group=None, target=self.threaded_disparity_map, name="Thread_num{}".format(i), args=(npzfile,))
            self.log.debug("Starting Thread_num {}".format(i))
            thread.start()

        # finally, rest the global interperter lock here:
        while True:
            if self.output_queue.empty() is not True:
                self.log.debug("displaying image!")
                jpg_image_bytes = self.output_queue.get()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + jpg_image_bytes + b'\r\n')
            time.sleep(0.5)

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
                self.log.debug("No more frames")
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
