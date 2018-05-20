import RPi.GPIO as GPIO
import time

class Camera():

    def __init__(self):
        self.servo_axis_x_pin = 11
        self.servo_axis_y_pin = 13

        x_axis_degrees = 0
        y_axis_degrees = 0
        # x_axis
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11,GPIO.OUT)
        self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
        self.pwm_x.start(1/18*(x_axis_degrees+90)+2)

        # y_axis
        GPIO.setup(13,GPIO.OUT)
        self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
        self.pwm_y.start(1/18*(y_axis_degrees+90)+2)

        time.sleep(1)
        self.pwm_x.stop()
        self.pwm_y.stop()
        return

    def move_camera(self, x_axis_degrees=None, y_axis_degrees=None):
        """
        # input in degrees, output in DutyCycle
        # pwm_x 0 is neutral, 90 is to the right, -90 is to the left
        # pwm_y 0 is neutral, 90 is up, -90 is down
        """
        if x_axis_degrees != None:
            self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
            self.pwm_x.start(1/18*((x_axis_degrees*-1)+90)+2)
        if y_axis_degrees != None:
            self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
            self.pwm_y.start(1/18*(y_axis_degrees+90)+2)
        """GPIO movement is not thread-blocking, so we must sleep thread"""
        time.sleep(0.75)
        self.stop_servos()

    def move_camera_slow(self, x_start, y_start, x_end, y_end):
        """
        First move x_axis, then move y_axis
        TODO: Refactor this to move both axises at the same time.
        """
        for i in range(x_start, x_end, 2):
            self.move_camera(i, None)
            time.sleep(0.1)
            print("Stopping servos...")
            self.stop_servos()
            time.sleep(0.1)

        for i in range(y_start, y_end, 2):
            self.move_camera(None, i)
            time.sleep(0.1)
            print("Stopping servos...")
            self.stop_servos()
            time.sleep(0.1)





        self.stop_servos()

    def stop_servos(self):
        self.pwm_x.stop()
        self.pwm_y.stop()


    def start_cameras(self):
        """
        # This method is used to start the cameras and display the video feed
        # on the GUI.

        Note: Imports are done in the method level because of latency issues with
        importing large modules on the RPi.
        """

        import cv2

        CAMERA_WIDTH = 1920
        CAMERA_HEIGHT = 1080

        left = cv2.VideoCapture(0)
        left.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        left.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        left.set(cv2.CAP_PROP_FPS,1)
        left.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        right = cv2.VideoCapture(1)
        right.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        right.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        right.set(cv2.CAP_PROP_FPS,1)
        right.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        while(True):
            if not (left.grab()):
                print("No more frames")
                break

            _, leftFrame = left.retrieve()
            _, rightFrame = right.retrieve()

            cv2.imshow('left', leftFrame)
            cv2.imshow('right', rightFrame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        left.release()
        right.release()
        cv2.destroyAllWindows()
