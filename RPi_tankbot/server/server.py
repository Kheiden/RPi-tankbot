from flask import Flask, request
import numpy as np
import cv2


class Server():

    def __init__(self):
        self.api_version = "v1"
        self.home_dir = "/home/kheiden"

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


    def start_webserver(self):
        print("Initializing Server")
        app = Flask(__name__)

        @app.route("/{}/disparitymap".format(self.api_version))
        def disparitymap():
            if request.method == 'POST':
                imgRGB_left = request.form['imgRGB_left']
                imgRGB_right = request.form['imgRGB_right']
                _, disparity = self.create_disparity_map(imgRGB_left,
                                                                imgRGB_right)

                response = Response(disparity, status=200,
                                    mimetype='application/octet-stream')
                return response
            else:
                return "Request needs to be POST"

        @app.route("/{}/serveronline".format(self.api_version))
        def server_online():
            return "ok"

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
    app.run(host='0.0.0.0', port=7842)
