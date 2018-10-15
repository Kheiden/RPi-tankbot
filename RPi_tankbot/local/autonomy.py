import movement
import camera

class Autonomy():

    def __init__(self):
        """ DEPRICATED. This function and module will be implemented on
        server/ not local/
        """

        self.c = camera.Camera()
        self.m = movement.Movement()

    def autonomous_routine_basic(self):
        """ DEPRICATED. This function and module will be implemented on the server

        This test is used to run a basic autonomous routine which will perform
        the following:
          1) Capture disparity map
          2) Move forward
          3) Repeat
          If the robot detects an object being too close, it will rotate an
          arbritary amount of time either left or right then continue with step 1
        """
        time_on = 5
        action = "rotate_random"
        # Threshold is the value between 0 and 255 that a pixel needs to be above
        # in order to count as being "too close"
        #Default threshold = 200
        threshold = 220
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

        return True


    def collision_avoidance_basic(self):
        """ DEPRICATED. This function and module will be implemented on the server


        This test will be used to determine if the RPi is about to run into a
        physical object.  This will be used by the autonomous routine to determine
        when to stop the robot.
        This test is comprised of a few parts.  First, the RPi takes a disparity
        map photo.  Second, the RPi checks if the disparity_map has pixels which
        are above the threshold, effectively proving that there is an object which
        is too close to the robot. Third, the method will send a shutdown function to
        the RPi to cease all movement.
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
        action = [threshold, num_threshold, action]
        movement_time = 0.50
        sleep_time = 2.00
        self.m.forward_slow_thread(movement_time, sleep_time)
        _, _, action = self.c.realtime_disparity_map_stream(time_on=time_on,
            action=action,
            save_disparity_image=True,
            override_warmup=False)
        self.m.stop()

        return True
