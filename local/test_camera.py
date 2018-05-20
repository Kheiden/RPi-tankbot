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

    @pytest.mark.skip(reason="Skipping to isolate test.")
    def test_camera_rotation(self):
        for i in range(-90, 91, 10):
            print("Moving camera to {}".format(i))
            self.c.move_camera(i, i)
            time.sleep(0.5)
            print("Stopping servos...")
            self.c.stop_servos()
            time.sleep(0.5)
        self.assertTrue(True)

    @pytest.mark.skip(reason="Skipping to isolate test.")
    def test_degree(self):
        x_axis_degrees = 0
        y_axis_degrees = 0

        print("Moving x_axis to {} and y_axis to {}".format(
            x_axis_degrees,
            y_axis_degrees))
        self.c.move_camera(x_axis_degrees, y_axis_degrees)

    #@pytest.mark.skip(reason="Testing.")
    def test_smooth_rotate(self):
        for i in range(-90, 91, 2):
            print("Moving camera to {}".format(i))
            self.c.move_camera(i, i)
            time.sleep(0.1)
            print("Stopping servos...")
            self.c.stop_servos()
            time.sleep(0.1)


if __name__ == '__main__':
    main()
