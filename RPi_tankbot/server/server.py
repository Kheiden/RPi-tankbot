from flask import Flask, request, send_file
import numpy as np
import movement
import pickle
import cv2
import io


class Server():

    def __init__(self):
        self.api_version = "v1"
        self.home_dir = "/home/kheiden"
        #self.m = movement.Movement()

    def create_disparity_map(self, imgLeft, imgRight, res_x=640, res_y=480, npzfile=None, ):
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

        if npzfile is None:
            npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

        imageSize = tuple(npzfile['imageSize'])
        leftMapX = npzfile['leftMapX']
        leftMapY = npzfile['leftMapY']
        rightMapX = npzfile['rightMapX']
        rightMapY = npzfile['rightMapY']

        width_left, height_left = imgLeft.shape[:2]
        width_right, height_right = imgRight.shape[:2]
        if 0 in [width_left, height_left, width_right, height_right]:
            print("Error: Can't remap image.")

        imgLeft = cv2.remap(imgLeft, leftMapX, leftMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgRight = cv2.remap(imgRight, rightMapX, rightMapY, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        grayLeft = cv2.cvtColor(imgLeft,cv2.COLOR_BGR2GRAY)
        grayRight = cv2.cvtColor(imgRight,cv2.COLOR_BGR2GRAY)

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

        # Compute the disparity image
        disparity = stereo.compute(grayLeft, grayRight)
        # Normalize the image for representation
        norm_coeff = 255 / disparity.max()
        disparity_normalized = disparity * norm_coeff / 255

        return imgLeft, disparity

    def move_robot(self, action):
        threshold = action[0]
        num_threshold = action[1]
        b = np.where(result[1] > threshold)
        num_pixels_above_threshold = len(b[0])
        print("Threshold: {}/255, num_threshold: {}, num_pixels_above_threshold: {}".format(
            threshold,
            num_threshold,
            num_pixels_above_threshold
        ))
        if num_pixels_above_threshold >= num_threshold:
            # This means that we need to stop the robot ASAP
            print("Object detected too close!")
            if action[2] == 'stop_if_close':
                #self.m.stop()
                print("Stopping robot to avoid collision")
                return None, None, action[2]

            if action[2] == 'rotate_random':
                direction = random.choice(["right", "left"])
                print("Rotating {} to avoid obstacle".format(direction))
                #move left or right
                #self.m.rotate_on_carpet(direction=direction,
                    #movement_time=6,
                    #sleep_speed=0.25)

            if action[2] == 'rotate_right':
                direction = "right"
                print("Rotating {} to avoid obstacle".format(direction))
                #move left or right
                #self.m.rotate_on_carpet(direction=direction,
                    #movement_time=6,
                    #sleep_speed=0.25)

    def start_webserver(self):
        print("Initializing Server")
        app = Flask(__name__)

        @app.route("/{}/disparitymap".format(self.api_version), methods=['POST'])
        def disparitymap():
            payload = request.get_json(force=True)
            imgRGB_left = payload['imgRGB_left']
            imgRGB_right = payload['imgRGB_right']
            action = payload['action']
            _, disparity = self.create_disparity_map(imgRGB_left,
                                                            imgRGB_right)
            #self.move_robot(action)
            return "ok"


        @app.route("/{}/serveronline".format(self.api_version))
        def server_online():
            return "Robot Brain server is online."

        return app

    """
    def process_video_stream(self):
        # Step 1) retrieve RGB data for two photos (left and right)
        # Step 2) Perform stereo photo calculation to determine distance to nearest target -> value
        # Step 3) Perform object recognition on one of the two photos -> Log to database
        return
    """

if __name__ == '__main__':
    s = Server()
    app = s.start_webserver()
    app.debug=True
    app.run(host='0.0.0.0', port=8081)
