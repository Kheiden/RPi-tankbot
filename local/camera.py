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

    def move_camera(self, x_axis_degrees, y_axis_degrees):
        """
        # input in degrees, output in DutyCycle
        # pwm_x 0 is neutral, 90 is to the right, -90 is to the left
        # pwm_y 0 is neutral, 90 is up, -90 is down
        """

        self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
        num1 = (((float(x_axis_degrees) + 90)/18) + 2)
        print(num1, x_axis_degrees)
        self.pwm_x.start(num1)
        self.pwm_x.ChangeDutyCycle(num1)

        self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
        num2 = (((float(y_axis_degrees) + 90)/18) + 2)
        print(num2, y_axis_degrees)
        self.pwm_y.start(num2)
        self.pwm_y.ChangeDutyCycle(num2)

        """
        # after moving, stop the servo acuation
        """
        time.sleep(1)
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
