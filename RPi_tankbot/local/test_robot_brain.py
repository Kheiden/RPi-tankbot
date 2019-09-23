import numpy as np
import robot_brain
import movement
import camera
import pytest

class TestBrain():

    @classmethod
    def setup_class(self):
        self.m = movement.Movement()
        self.c = camera.Camera()
        self.home_dir = "/home/pi"
        self.brain = robot_brain.RobotBrain()

    @classmethod
    def teardown_class(self):
        return

    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_server_online(self):
        # First test to make sure that the server is online
        result = self.brain.server_alive()
        assert result.status_code == 200

    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_server_compute_disparity_map(self):
        """
        This test is used to compute the disparity maps by using an external
        server. The server will be running the code in ../server/server.py

        In order to test this code, I need to ensure that portforwarding is
        active on 192.168.1.10:7842 -> <Robot Brain IP address>

        """
        # First test to make sure that the server is online
        result = self.brain.server_alive()
        assert result.status_code == 200

        list_of_results = []
        number_of_tests = 1
        processing_speed_threshold = 1

        time_on = 5
        action = "stop_if_close"
        # Threshold is the value between 0 and 255 that a pixel needs to be above
        # in order to count as being "too close"
        threshold = 200
        # num_threshold is the number of pixels that are above the threshold
        # 640*480 = 307200
        # 640*480*0.05 = 15360
        # 640*480*0.10 = 30720
        # 5 percent: 4'6" to 4'10" away from target
        # 10 percent: <tbd>
        num_threshold = 30720
        action = [threshold, num_threshold, action]

        print("Pretending to move the robot forward")
        #self.m.forward_slow(1, 1)
        # Running the test a number of times to get the computation times
        for i in range(number_of_tests):
            # Now that we know that the server is online, we can compute a disparity
            # map
            processing_time = self.c.server_compute_disparity_map(action)
            print("server_compute_disparity_map() processing_time:", processing_time)
            list_of_results.append(processing_time)

        print("list_of_results:", list_of_results)

        # Below test fails if any of the results are above the threshold
        # If test is less than threshold then True, else False
        assert any([i for i in list_of_results if i < processing_speed_threshold])
