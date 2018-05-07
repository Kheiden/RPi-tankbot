import unittest
import camera
import time

class TestCamera(unittest.TestCase):

    def setUp(self):
        self.c = camera.Camera()

    def tearDown(self):
        self.c.stop_servos()

    @unittest.skip("")
    def test_camera_rotation(self):
        for i in range(-90, 91, 10):
            print("Moving camera to {}".format(i))
            self.c.move_camera(i, i)
            time.sleep(0.5)
            print("Stopping servos...")
            self.c.stop_servos()
            time.sleep(0.5)
        self.assertTrue(True)

    @unittest.skip("")
    def test_degree(self):
        x_axis_degrees = 0
        y_axis_degrees = 0

        print("Moving x_axis to {} and y_axis to {}".format(
            x_axis_degrees,
            y_axis_degrees))
        self.c.move_camera(x_axis_degrees, y_axis_degrees)

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
