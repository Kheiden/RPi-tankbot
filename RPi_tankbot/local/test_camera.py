import robot_brain

import numpy as np
import pytest

import movement
import camera
import time
import glob
import cv2
import io

import RPi.GPIO as GPIO

from PIL import Image

class TestCamera():

    @classmethod
    def setup_class(self):
        self.m = movement.Movement()
        self.c = camera.Camera()
        self.home_dir = "/home/pi"
        self.brain = robot_brain.RobotBrain()

    @classmethod
    def teardown_class(self):
        self.c.stop_servos()

    @pytest.mark.skip(reason="")
    def test_rotate_new(self):
      GPIO.setmode(GPIO.BOARD)
      GPIO.setwarnings(False)
      coil_A_1_pin = 15 # pink (Actual color: red)
      coil_A_2_pin = 11 # orange (Actual color: black)
      coil_B_1_pin = 13 # blue (Actual color: blue)
      coil_B_2_pin = 12 # yellow(Actual color: green)

      # adjust if different
      StepCount = 8
      Seq = range(0, StepCount)
      #     BLK,RED,GRN,BLU]
      #       [A,B,A\,B\]
      Seq[0] = [1,1,0,0]
      Seq[1] = [0,1,1,0]
      Seq[2] = [0,0,1,1]
      Seq[3] = [1,0,0,1]

      # GPIO.setup(enable_pin, GPIO.OUT)
      GPIO.setup(coil_A_1_pin, GPIO.OUT)
      GPIO.setup(coil_A_2_pin, GPIO.OUT)
      GPIO.setup(coil_B_1_pin, GPIO.OUT)
      GPIO.setup(coil_B_2_pin, GPIO.OUT)

      # GPIO.output(enable_pin, 1)

      def setStep(w1, w2, w3, w4):
        GPIO.output(coil_A_1_pin, w1)
        GPIO.output(coil_A_2_pin, w2)
        GPIO.output(coil_B_1_pin, w3)
        GPIO.output(coil_B_2_pin, w4)

      def forward(delay, steps):
        for i in range(steps):
            for j in range(StepCount):
                setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
                time.sleep(delay)

      def backwards(delay, steps):
        for i in range(steps):
            for j in reversed(range(StepCount)):
                setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
                time.sleep(delay)

      if __name__ == '__main__':
        while True:
            delay = raw_input("Time Delay (ms)?")
            steps = raw_input("How many steps forward? ")
            forward(int(delay) / 1000.0, int(steps))
            steps = raw_input("How many steps backwards? ")
            backwards(int(delay) / 1000.0, int(steps))

    @pytest.mark.skip(reason="Passed.")
    def test_basic_autonomous_routine(self):
        """
        DEPRICATED: Use autonomy.py instead
        """
        time_on = 5
        action = "rotate_random"
        # Threshold is the value between 0 and 255 that a pixel needs to be above
        # in order to count as being "too close"
        threshold = 200
        # num_threshold is the number of pixels that are above the threshold
        # 640*480 = 307200
        # 640*480*0.05 = 15360
        # 640*480*0.01 = 30720
        # 5 percent: 4'6" to 4'10" away from target
        # 10 percent: <tbd>
        num_threshold = 30720
        action = [threshold, num_threshold, action]

        processing_time, frame_counter, action = self.c.realtime_disparity_map_stream(time_on=time_on,
            action=action,
            save_disparity_image=True,
            override_warmup=False,
            autonomous_routine="basic")

    @pytest.mark.skip(reason="Passed.")
    def test_collision_avoidance(self):
        """
        DEPRICATED: Use autonomy.py instead
        """
        # Max time on
        time_on = 30
        action = "stop_if_close"
        # Threshold is the value between 0 and 255 that a pixel needs to be above
        # in order to count as being "too close"
        threshold = 200
        # num_threshold is the number of pixels that are above the threshold
        # 640*480 = 307200
        # 640*480*0.05 = 15360
        # 640*480*0.01 = 30720
        # 5 percent: 4'6" to 4'10" away from target
        # 10 percent: <tbd>
        num_threshold = 30720
        action = [threshold, num_threshold]
        movement_time = 0.50
        sleep_time = 2.00
        self.m.forward_slow_thread(movement_time, sleep_time)
        _, _, action = self.c.realtime_disparity_map_stream(time_on=time_on,
            action=action,
            save_disparity_image=True,
            override_warmup=False)
        # %5 error tolerance for the stream to be on
        if action == 'stop_robot':
            print("Stopping robot.")
            self.m.stop()

        assert action is not None

    @pytest.mark.skip(reason="Passed.")
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
                type="image_array",
                override_warmup=True,
                quick_capture=True)
            type_left = type(imgBGR_left)
            type_right =type(imgBGR_right)
            #print(type_left, type_right)
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

        print("processing_time:", processing_time)
        print("frame_counter", frame_counter)
        print("frames per second:", (frame_counter/processing_time))
        print("fake_frames, real_frames", fake_frames, real_frames)
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

    @pytest.mark.skip(reason="Test Failing.")
    def test_camera_frames(self):
        time_on = 30
        frame_counter = 0
        fps = 2
        res_x = 640
        res_y = 480
        processing_time01 = cv2.getTickCount()
        while True:
            result = self.c.take_stereo_photo(res_x, res_y, type="image_array", override_warmup=True)
            processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
            frame_counter += 1
            if processing_time >= time_on:
                break
        print("processing_time:", processing_time)
        print("frame_counter", frame_counter)
        assert processing_time <= (time_on * 1.05)
        assert frame_counter >= time_on * fps
        #2.2 seconds per frame (14 frames)

    @pytest.mark.skip(reason="Test Failing.")
    def test_realtime_disparity_map_stream(self):
        # specify the amount of time that the stream is open for
        time_on = 3
        # Target FPS
        fps = 2
        processing_time, frame_counter = self.c.realtime_disparity_map_stream(time_on=time_on)
        # %5 error tolerance for the stream to be on
        print("Processing time:{} number of frames:{}".format(processing_time, frame_counter))
        assert processing_time <= (time_on * 1.05)
        assert frame_counter >= time_on * fps
        #1) 6.18 seconds per frame (1 frame)
        #2) 5.39 seconds per frame (1 frame)
        #3) 4.46 seconds per frame (7 frames)
        #4) 3.36 frames per second (9 frames)

    @pytest.mark.skip(reason="Passed.")
    def test_create_3d_point_cloud(self):
        res_x = 640
        res_y = 480
        imgL, imgR = self.c.take_stereo_photo(res_x, res_y, type="image_array")
        assert imgL.any()
        assert imgR.any()
        imgLeft, disparity_map = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)
        result = self.c.create_3d_point_cloud(imgLeft, disparity_map)
        assert result

    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_create_multiple_disparity_maps(self):
        """
        This test is used to create multiple disparity maps so that I can
        average them all into a single disparity map
        """
        res_x = 640
        res_y = 480
        # Used to denote how many disparity maps will be taken
        number_of_pictures = 25

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        # Look for the calib data
        npzfile = np.load('{}/calibration_data/{}p/stereo_camera_calibration.npz'.format(self.home_dir, res_y))

        for calib_data in range(number_of_pictures):
            imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, None, type="image_array", quick_capture=False)
            if type(imgL) is type(None):
                print("Problem taking image")
                assert False

            if type(imgR) is type(None):
                print("Problem taking image")
                assert False

                result = self.c.create_disparity_map(imgL, imgR, res_x=640,
                                                    res_y=480,
                                                    save_disparity_image=True,
                                                    npzfile=npzfile)
                assert result

        right.release()
        left.release()

    @pytest.mark.skip(reason="Passed.")
    def test_create_disparity_maps_with_multiple_calib_data(self):
        """
        This test is used to create multiple disparity maps from different
        camera calibration datasets.

        The ultimate goal is to determine how effective the camera calibration is
        with the quality of the disparity map.
        """
        res_x = 640
        res_y = 480
        # Used to denote how many disparity maps will be taken for each calib data
        number_of_pictures = 3

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # Look for the backup calib data
        all_calib_data = glob.glob('{}/backup_calibration_data/{}p/stereo_camera_calibration*.npz'.format(self.home_dir, res_y))

        for calib_data in all_calib_data:
            npzfile = np.load(calib_data)
            for i in range(number_of_pictures):
                imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, None, type="image_array", quick_capture=False)
                if type(imgL) is type(None):
                    print("Problem taking image")
                    assert False

                if type(imgR) is type(None):
                    print("Problem taking image")
                    assert False

                result = self.c.create_disparity_map(imgL, imgR, res_x=640,
                                                    res_y=480,
                                                    save_disparity_image=True,
                                                    npzfile=npzfile)
                assert result

        right.release()
        left.release()

    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_create_single_disparity_map(self):
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


        # Below images are BGR
        imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, None, type="image_array", quick_capture=False)

        if type(imgL) is type(None):
            print("Problem taking image")
            assert False

        if type(imgR) is type(None):
            print("Problem taking image")
            assert False

        result = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)
        assert result

        right.release()
        left.release()

    @pytest.mark.skip(reason="Passed.")
    def test_undistort_image_multiple_resolution(self):
        """
        # I want to be able to undistort an image in less than 1 second
        """
        threshold_miliseconds = 1000
        #['270p', "540p", "1080p"]
        for resolution in ['480p']:
            img = cv2.imread('{}/input_output/{}/input_left.jpg'.format(self.home_dir, resolution))

            result1 = self.c.undistort_image(img=img, cam_num=0)
            print(result1[0]*1000)
            assert (result1[0]*1000 < threshold_miliseconds)
            cv2.imwrite('{}/input_output/{}/output_left.jpg'.format(self.home_dir, resolution), np.hstack((img, result1[1])))
            img = cv2.imread('{}/input_output/{}/input_right.jpg'.format(self.home_dir, resolution))
            assert (result1[0]*1000 < threshold_miliseconds)
            result2 = self.c.undistort_image(img=img, cam_num=1)
            print(result2[0]*1000)
            assert (result2[0]*1000 < threshold_miliseconds)
            cv2.imwrite('{}/input_output/{}/output_right.jpg'.format(self.home_dir, resolution), np.hstack((img, result2[1])))

    @pytest.mark.skip(reason="Passed.")
    def test_calibrate_stereo_camera(self):
        threshold_seconds = 30
        result1 = self.c.calibrate_camera(cam_num=0)
        print(result1)
        assert (result1 < threshold_seconds)
        result2 = self.c.calibrate_camera(cam_num=1)
        print(result2)
        assert (result2 < threshold_seconds)
        result3 = self.c.calibrate_stereo_cameras()

    @pytest.mark.skip(reason="Passed.")
    def test_calibrate_cameras(self):
        """
        # Calibration takes about half an hour for each camera
        # if the calibration data exists, then the method will return
        """
        threshold_seconds = 1800
        result1 = self.c.calibrate_camera(cam_num=0, res_x=1920, res_y=1080)
        print(result1)
        assert (result1 < threshold_seconds)
        result2 = self.c.calibrate_camera(cam_num=1, res_x=1920, res_y=1080)
        print(result2)
        assert (result2 < threshold_seconds)

    @pytest.mark.skip(reason="Passed.")
    def test_chessboard_photos(self):
        res_x = 640 #1920
        res_y = 480 #1080
        for i in range(15):
            width, height = self.c.take_stereo_photo(res_x, res_y, type="separate")
            assert width == res_x
            assert height == res_y

    @pytest.mark.skip(reason="Passed.")
    def test_take_picture(self):
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

        ret_left = left.grab()
        ret_right = right.grab()
        _, rightFrame = right.retrieve()
        _, leftFrame = left.retrieve()

        assert ret_left
        assert ret_right

        right.release()
        left.release()

    @pytest.mark.skip(reason="Passed.")
    def test_take_stereo_photo_yield(self):
      assert self.c.take_stereo_photo_yield()

    @pytest.mark.skip(reason="Passed.")
    def test_stereo_photo_save_to_disk(self):
      assert self.c.stereo_photo_save_to_disk()

    @pytest.mark.skip(reason="Passed.")
    def test_stereo_photo_new(self):
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

            # Below images are BGR
        imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, None, type="image_array", quick_capture=True)

        assert imgL is not None
        assert imgR is not None

        right.release()
        left.release()

    @pytest.mark.skip(reason="Passed.")
    def test_stereo_photo(self):
        res_x = 1920
        res_y = 1080
        width, height = self.c.take_stereo_photo(res_x, res_y, type="combined")
        assert width == res_x*2
        assert height == res_y

        res_x = 1920
        res_y = 1080
        width, height = self.c.take_stereo_photo(res_x, res_y, type="separate")
        assert width == res_x
        assert height == res_y

    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_move_camera(self):
      # First start at -90 degrees
      self.c.move_camera(x_axis_degrees=-90, y_axis_degrees=-90)
      self.c.move_camera(x_axis_degrees=90, y_axis_degrees=90)

if __name__ == '__main__':
    main()
