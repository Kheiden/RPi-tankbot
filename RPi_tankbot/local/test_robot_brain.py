import camera
import robot_brain

class TestBrain():

    @classmethod
    def setup_class(self):
        self.c = camera.Camera()
        self.home_dir = "/home/pi"
        self.brain = robot_brain.RobotBrain()

    @classmethod
    def teardown_class(self):
        return

    def test_server_online(self):
        # First test to make sure that the server is online
        result = self.brain.server_alive()
        assert result.status_code == 200

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
        number_of_tests = 10
        threshold = 1

        # Running the test a number of times to get the
        for i in range(number_of_tests):
            # Now that we know that the server is online, we can compute a disparity
            # map
            processing_time, disparity_map = self.c.server_compute_disparity_map()
            print("server_compute_disparity_map() processing_time:", processing_time)
            list_of_results.append(processing_time)

        print("list_of_results:", list_of_results)
        # Below test fails if any of the results are above the threshold
        assert any([i for i in list_of_results if i > threshold])
