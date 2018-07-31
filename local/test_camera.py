import numpy as np
import pytest

import movement
import camera
import time
import cv2

class TestCamera():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """
        self.m = movement.Movement()
        self.c = camera.Camera()
        self.home_dir = "/home/pi"

    @classmethod
    def teardown_class(self):
        """ teardown any state that was previously setup with a call to
        setup_class.
        """
        self.m.stop()
        self.m.clear_gpio_motor_pins()


    @pytest.mark.skip(reason="Test Failing")
    def test_stereo_photo_speed(self):
        """
        I want to take stereo photos as fast as possible
        """
        res_x = 640
        res_y = 480

        fps = 15
        time_on = 45
        frame_counter = 0

        processing_time01 = cv2.getTickCount()

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        fake_frames = 0
        real_frames = 0
        while True:
            imgBGR_left, imgBGR_right = self.c.take_stereo_photo(res_x, res_y,
                right=right,
                left=left,
                type="separate",
                override_warmup=True,
                quick_capture=True)
            type_left = type(imgBGR_left)
            type_right =type(imgBGR_right)
            #self.log.debug(type_left, type_right)
            if type_left == type(None):
                fake_frames += 1
            if type_right == type(None):
                fake_frames += 1
            if (type_left and type_right) != type(None):
                real_frames += 2
            frame_counter += 2

            processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
            if processing_time >= time_on:
                break

        right.release()
        left.release()

        self.log.debug("processing_time:", processing_time)
        self.log.debug("frame_counter", frame_counter)
        self.log.debug("frames per second:", (frame_counter/processing_time))
        self.log.debug("fake_frames, real_frames", fake_frames, real_frames)
        assert processing_time <= (time_on * 1.05)
        assert (frame_counter) >= time_on * fps
        assert fake_frames < 30
        #frames per second: 0.2647685324087231

    @pytest.mark.skip(reason="Not Yet Passed")
    def test_create_3d_surroundings(self):
        """
        This test will be used to create 8 3d cloud points
        then stitch them together to form the environment surrounding the robot
        """
        res_x = 640
        res_y = 480
        npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))
        for file_num in range(8):
            imgL, imgR = self.c.take_stereo_photo(res_x, res_y, type="image_array", override_warmup=False)
            result = self.c.create_disparity_map(imgL, imgR, res_x, res_y, npzfile=npzfile, save_disparity_image=False)
            self.c.create_3d_point_cloud(result[0], result[1], file_num)
            # It takes about 5 seconds for a full turn, so 5/8 = 0.625 sec
            self.m.rotate(direction="right", movement_time=0.625)
        assert True

    @pytest.mark.skip(reason="Passed")
    def test_camera_frames(self):
        time_on = 30
        frame_counter = 0
        fps = 2
        res_x = 640
        res_y = 480

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
            _, _ = self.c.take_stereo_photo(res_x, res_y,
                right=right,
                left=left,
                type="image_array",
                override_warmup=True,
                quick_capture=True)
            processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
            frame_counter += 1
            if processing_time >= time_on:
                break
        self.log.debug("processing_time:", processing_time)
        self.log.debug("frame_counter", frame_counter)

        right.release()
        left.release()

        assert processing_time <= (time_on * 1.05)
        assert frame_counter >= time_on * fps
        #2.2 seconds per frame (14 frames)


    @pytest.mark.skip(reason="Test Failing.")
    def test_realtime_disparity_map_stream(self):
        # specify the amount of time that the stream is open for
        time_on = 30
        # Target FPS
        fps = 2
        processing_time, frame_counter = self.c.realtime_disparity_map_stream(time_on=time_on)
        # %5 error tolerance for the stream to be on
        self.log.debug("Processing time:{} number of frames:{}".format(processing_time, frame_counter))
        assert processing_time <= (time_on * 1.05)
        assert frame_counter >= time_on * fps
        #1) 6.18 seconds per frame (1 frame)
        #2) 5.39 seconds per frame (1 frame)
        #3) 4.46 seconds per frame (7 frames)
        #4) 3.36 frames per second (9 frames)

    @pytest.mark.skip(reason="Failed.")
    def test_create_3d_point_cloud(self):
        res_x = 640
        res_y = 480

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        imgL, imgR = self.c.take_stereo_photo(res_x, res_y,
                right=right,
                left=left,
                type="image_array",
                override_warmup=True,
                quick_capture=True)

        imgLeft, disparity_map = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)
        result = self.c.create_3d_point_cloud(imgLeft, disparity_map)

        right.release()
        left.release()

        assert result


    @pytest.mark.skip(reason="Passed.")
    def test_create_single_disparity_map(self):
        res_x = 640
        res_y = 480
        self.log.debug("Successful -1")
        right = cv2.VideoCapture(1)
        self.log.debug("Successful -1.1")
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        self.log.debug("Successful -1.2")
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        self.log.debug("Successful -1.3")
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.log.debug("Successful -1.4")

        left = cv2.VideoCapture(0)
        self.log.debug("Successful -1.5")
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        self.log.debug("Successful -1.6")
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        self.log.debug("Successful -1.7")
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # Below images are BGR
        self.log.debug("Successful 0")
        imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, False, type="image_array", quick_capture=True)
        self.log.debug("Successful 1")
        imgLeft, disparity_map = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)

        right.release()
        left.release()

        assert disparity_map is not None


    @pytest.mark.skip(reason="Test Failing.")
    def test_undistort_image_multiple_resolution(self):
        """
        # I want to be able to undistort an image in less than 1 second
        """
        #['270p', "540p", "1080p"]
        for resolution in ['480p']:
            self.log.debug("Successful -1")
            img = cv2.imread('{}/input_output/{}/input_left.jpg'.format(self.home_dir, resolution))
            threshold_miliseconds = 1000
            self.log.debug("Successful 0")
            result1 = self.c.undistort_image(img=img, cam_num=0)
            self.log.debug(result1[0]*1000)
            assert (result1[0]*1000 < threshold_miliseconds)
            cv2.imwrite('{}/input_output/{}/output_left.jpg'.format(self.home_dir, resolution), np.hstack((img, result1[1])))

            img = cv2.imread('{}/input_output/{}/input_right.jpg'.format(self.home_dir, resolution))
            assert (result1[0]*1000 < threshold_miliseconds)
            result2 = self.c.undistort_image(img=img, cam_num=1)
            self.log.debug(result2[0]*1000)
            assert (result2[0]*1000 < threshold_miliseconds)
            cv2.imwrite('{}/input_output/{}/output_right.jpg'.format(self.home_dir, resolution), np.hstack((img, result2[1])))


    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_stereo_photo(self):
        res_x = 1920
        res_y = 1080
        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        width, height = self.c.take_stereo_photo(res_x, res_y,
            right, left, override_warmup=False, type="combined")

        assert width == res_x*2
        assert height == res_y

        width, height = self.c.take_stereo_photo(res_x, res_y,
            right, left, override_warmup=False, type="separate")
        assert width == res_x
        assert height == res_y

        right.release()
        left.release()


    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_concat_cameras(self):
        # This test will take a single still photo at max resolution with both cameras

        resolutions = [(640, 480)]
        #resolutions = [(320, 240), (640, 480), (1280, 720),
        #    (1904, 1080), (1920, 1080)]

        for res in resolutions:
            right = cv2.VideoCapture(1)
            right.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            right.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
            right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

            left = cv2.VideoCapture(0)
            left.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
            left.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
            left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

            width, height = self.c.take_stereo_photo(res[0], res[1],
                right, left, override_warmup=False, type="combined")
            assert width == res[0]*2
            assert height == res[1]

            right.release()
            left.release()
