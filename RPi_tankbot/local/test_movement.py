import pytest

import time
import movement

class TestMovement():

    @classmethod
    def setup_class(self):
        self.m = movement.Movement()

    @classmethod
    def teardown_class(self):
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


    @pytest.mark.skip(reason="Passed.")
    def test_move_robot(self):
        """move forward, turn, then move forward again."""
        self.m.forward(movement_time=3)
        self.m.backward(movement_time=3)
        self.m.rotate(direction="right", movement_time=3)
        self.m.rotate(direction="left", movement_time=3)

    @pytest.mark.skip(reason="Passed.")
    def test_infinite_motor_movement(self):
        """Move motors without passing movement_time variable"""
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

    @pytest.mark.skip(reason="Not yet passed.")
    def test_camera_rotation(self):
        for i in range(-90, 91, 10):
            print("Moving camera to {}".format(i))
            self.m.move_camera(i, i)
            time.sleep(0.5)
            print("Stopping servos...")
            self.m.stop_servos()
            time.sleep(0.5)
        self.assertTrue(True)

    @pytest.mark.skip(reason="Not yet passed.")
    def test_degree(self):
        x_axis_degrees = 0
        y_axis_degrees = 0

        print("Moving x_axis to {} and y_axis to {}".format(
            x_axis_degrees,
            y_axis_degrees))
        self.m.move_camera(x_axis_degrees, y_axis_degrees)


    @pytest.mark.skip(reason="Not yet passed.")
    def test_smooth_rotate(self):
        """First, center the camera"""
        self.m.move_camera(0, 0)
        """Then, move to a certain coordinate"""
        x_start = 0
        y_start = 0
        x_end = 45
        y_end = 30
        """Testing Slow speed"""
        speed = "SLOW"
        self.m.move_camera_smooth(x_start, y_start, x_end, y_end, speed)
        """Reset Camera to zero"""
        sself.m.move_camera(0, 0)
        """Testing fast speed"""
        speed = "FAST"
        self.m.move_camera_smooth(x_start, y_start, x_end, y_end, speed)

    @pytest.mark.skip(reason="Not yet passed.")
    def test_zero_camera(self):
        self.m.move_camera(0, 0)
