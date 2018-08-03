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
        self.c.stop_servos()

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
        time_on = 30
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
        imgLeft, disparity_map = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)
        result = self.c.create_3d_point_cloud(imgLeft, disparity_map)
        assert result


    @pytest.mark.skip(reason="Passed.")
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

        for i in range(45):
            # Below images are BGR
            imgL, imgR = self.c.take_stereo_photo(res_x, res_y, right, left, False, type="image_array", quick_capture=True)
            result = self.c.create_disparity_map(imgL, imgR, res_x=640, res_y=480, save_disparity_image=True)
            assert result


    @pytest.mark.skip(reason="Passed.")
    def test_undistort_image_multiple_resolution(self):
        """
        # I want to be able to undistort an image in less than 1 second
        """
        #['270p', "540p", "1080p"]
        for resolution in ['480p']:
            img = cv2.imread('{}/input_output/{}/input_left.jpg'.format(self.home_dir, resolution))
            threshold_miliseconds = 1000
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

    @pytest.mark.skip(reason="Passed.")
    def test_concat_cameras(self):
        # This test will take a single still photo at max resolution with both cameras
        res_x = 320
        res_y = 240
        width, height = self.c.take_stereo_photo(res_x, res_y, type="combined")
        assert width == res_x*2
        assert height == res_y

        res_x = 640
        res_y = 480
        width, height = self.c.take_stereo_photo(res_x, res_y, type="combined")
        assert width == res_x*2
        assert height == res_y

        res_x = 1280
        res_y = 720
        width, height = self.c.take_stereo_photo(res_x, res_y , type="combined")
        assert width == res_x*2
        assert height == res_y

        res_x = 1904
        res_y = 1080
        width, height = self.c.take_stereo_photo(res_x, res_y , type="combined")
        assert width == res_x*2
        assert height == res_y

        res_x = 1920
        res_y = 1080
        width, height = self.c.take_stereo_photo(res_x, res_y , type="combined")
        assert width == res_x*2
        assert height == res_y

    #@pytest.mark.skip(reason="Not Yet Passed.")
    def test_concat_cameras_new(self):
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

            if not right.isOpened() or not left.isOpened():
                print("Unable to access Cameras")
                assert False

            width, height = self.c.take_stereo_photo(res[0], res[1],
                right, left, override_warmup=False, type="combined")
            if width or height is None:
                # Fail test
                assert False
            assert width == res[0]*2
            assert height == res[1]

            right.release()
            left.release()


    @pytest.mark.skip(reason="Not yet passed.")
    def test_camera_rotation(self):
        for i in range(-90, 91, 10):
            print("Moving camera to {}".format(i))
            self.c.move_camera(i, i)
            time.sleep(0.5)
            print("Stopping servos...")
            self.c.stop_servos()
            time.sleep(0.5)
        self.assertTrue(True)

    @pytest.mark.skip(reason="Not yet passed.")
    def test_degree(self):
        x_axis_degrees = 0
        y_axis_degrees = 0

        print("Moving x_axis to {} and y_axis to {}".format(
            x_axis_degrees,
            y_axis_degrees))
        self.c.move_camera(x_axis_degrees, y_axis_degrees)


    @pytest.mark.skip(reason="Not yet passed.")
    def test_smooth_rotate(self):
        """First, center the camera"""
        self.c.move_camera(0, 0)
        """Then, move to a certain coordinate"""
        x_start = 0
        y_start = 0
        x_end = 45
        y_end = 30
        """Testing Slow speed"""
        speed = "SLOW"
        self.c.move_camera_smooth(x_start, y_start, x_end, y_end, speed)
        """Reset Camera to zero"""
        self.c.move_camera(0, 0)
        """Testing fast speed"""
        speed = "FAST"
        self.c.move_camera_smooth(x_start, y_start, x_end, y_end, speed)

    @pytest.mark.skip(reason="Not yet passed.")
    def test_zero_camera(self):
        self.c.move_camera(0, 0)


if __name__ == '__main__':
    main()
