import movement
import pytest
import time
import cv2

class TestMovement():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """

        self.m = movement.Movement()

    @classmethod
    def teardown_class(self):
        """ teardown any state that was previously setup with a call to
        setup_class.
        """
        self.m.clear_gpio_motor_pins()

    @pytest.mark.skip(reason="Passed.")
    def test_rotate_on_carpet(self):
        """Issue #68 on GitHub"""
        movement_time = 25
        sleep_speed = 0.25
        real_movement_time, num_cycles = self.m.rotate_on_carpet(direction="right",
            movement_time=movement_time,
            sleep_speed=sleep_speed)
        # real_movement_time needs to be between 3 and 3*1.05
        print(real_movement_time)
        assert real_movement_time <= (movement_time * 1.05)
        assert real_movement_time >= movement_time
        # This is the estimated number of cycles that the robot will perform
        # in the given amount of time
        print(num_cycles)
        assert num_cycles >= (movement_time / (sleep_speed * 2)) * 0.95

        """
        Results of test:
        real time: 10 seconds
        num_cycles: 50
        sleep_speed: 0.1
        degrees_rotated: <unknown-(need rotary encoder data)>

        real time: 25 seconds
        num_cycles: 50
        sleep_speed: 0.25
        degrees_rotated: <unknown-(need rotary encoder data)>
        estimated degrees rotated: estimated to around 270 degrees, but the left
            track came off of the front left wheel (not the drive wheel)
        """


    #@pytest.mark.skip(reason="Test Failing")
    def test_move_robot(self):
        """move forward, turn, then move forward again."""
        time_on = 12
        processing_time01 = cv2.getTickCount()
        self.m.forward(movement_time=3)
        self.m.backward(movement_time=3)
        self.m.rotate(direction="right", movement_time=3)
        self.m.rotate(direction="left", movement_time=3)
        processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
        print(processing_time)
        assert processing_time <= (time_on * 1.05)

    #@pytest.mark.skip(reason="Test Failing.")
    def test_infinite_motor_movement(self):
        """Move motors without passing movement_time variable"""
        time_on = 12
        processing_time01 = cv2.getTickCount()
        self.m.forward()
        time.sleep(3)
        self.m.stop()
        self.m.backward()
        time.sleep(3)
        self.m.stop()
        self.m.rotate(direction="right")
        time.sleep(3)
        self.m.stop()
        self.m.rotate(direction="left")
        time.sleep(3)
        self.m.stop()
        processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
        print(processing_time)
        assert processing_time <= (time_on * 1.05)

    @pytest.mark.skip(reason="Not yet passed.")
    def test_camera_servo_rotation_horizontal(self):
        # TODO- Update this test
        degrees = [-60, -45, -15, 0, 15, 45, 60]

        for degrees in list:
            output = self.c.take_stereo_photo()
            undistorted = self.c.undistort(output)
            location_1 = self.c.check_where_camera_is_pointing()

            self.m.move_servo_to_coords(degrees, 0)
            output = self.c.take_stereo_photo()
            undistorted = self.c.undistort(output)
            location_2 = self.c.check_where_camera_is_pointing()

            result = self.c.calculate_delta(location_1, location_2)
            assert result == degrees_to_be_rotated

    @pytest.mark.skip(reason="Not yet passed.")
    def test_camera_servo_rotation_vertical(self):
        # TODO- Update this test
        degrees = [-60, -45, -15, 0, 15, 45, 60]

        for degrees in list:
            output = self.c.take_stereo_photo()
            undistorted = self.c.undistort(output)
            location_1 = self.c.check_where_camera_is_pointing()

            self.m.move_servo_to_coords(0, degrees)
            output = self.c.take_stereo_photo()
            undistorted = self.c.undistort(output)
            location_2 = self.c.check_where_camera_is_pointing()

            result = self.c.calculate_delta(location_1, location_2)
            assert result == degrees_to_be_rotated


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
