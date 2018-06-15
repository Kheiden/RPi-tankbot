import pytest

import camera
import time

class TestCamera():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """

        self.c = camera.Camera()

    @classmethod
    def teardown_class(self):
        """ teardown any state that was previously setup with a call to
        setup_class.
        """
        self.c.stop_servos()


    def test_calibration_cameras(self):
        result1 = self.c.calibrate_camera(cam_num=0)
        assert any(result1)
        result2 = self.c.calibrate_camera(cam_num=1)
        assert any(result2)


    @pytest.mark.skip(reason="Passed.")
    def test_stereo_photo(self):
        x_res = 1920
        y_res = 1080
        width, height = self.c.take_stereo_photo(x_res, y_res , type="combined")
        assert width == x_res*2
        assert height == y_res

        x_res = 1920
        y_res = 1080
        width, height = self.c.take_stereo_photo(x_res, y_res , type="separate")
        assert width == x_res
        assert height == y_res

    @pytest.mark.skip(reason="Passed.")
    def test_concat_cameras(self):
        # This test will take a single still photo at max resolution with both cameras
        x_res = 320
        y_res = 240
        width, height = self.c.take_stereo_photo(x_res, y_res, type="combined")
        assert width == x_res*2
        assert height == y_res

        x_res = 640
        y_res = 480
        width, height = self.c.take_stereo_photo(x_res, y_res, type="combined")
        assert width == x_res*2
        assert height == y_res

        x_res = 1280
        y_res = 720
        width, height = self.c.take_stereo_photo(x_res, y_res , type="combined")
        assert width == x_res*2
        assert height == y_res

        x_res = 1904
        y_res = 1080
        width, height = self.c.take_stereo_photo(x_res, y_res , type="combined")
        assert width == x_res*2
        assert height == y_res

        x_res = 1920
        y_res = 1080
        width, height = self.c.take_stereo_photo(x_res, y_res , type="combined")
        assert width == x_res*2
        assert height == y_res

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
