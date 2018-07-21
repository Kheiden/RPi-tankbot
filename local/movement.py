import RPi.GPIO as GPIO
from time import sleep
import state
import cv2

class Movement():

    def __init__(self):
        # Inport the robot's state
        self.state = state.State()

        GPIO.setmode(GPIO.BOARD)
        # Motor1 is the left motor
        self.Motor1A = 16
        self.Motor1B = 18
        self.Motor1E = 22

        # Motor2 is the right motor
        self.Motor2A = 23
        self.Motor2B = 21
        self.Motor2E = 19

        GPIO.setup(self.Motor1A,GPIO.OUT)
        GPIO.setup(self.Motor1B,GPIO.OUT)
        GPIO.setup(self.Motor1E,GPIO.OUT)

        GPIO.setup(self.Motor2A,GPIO.OUT)
        GPIO.setup(self.Motor2B,GPIO.OUT)
        GPIO.setup(self.Motor2E,GPIO.OUT)


    def rotate_on_carpet(self, direction=None, movement_time=None, sleep_speed=0.25):
        self.state.stopped = False
        """
        direction: "left" or "right"
        movement_time: The total duration of movement
        sleep_speed: used to determine how fast the robot switches between
            sleeping the motors, effectively determining how long one motor is on
            before shutting it off and letting the other motor take over
        """
        processing_time01 = cv2.getTickCount()
        num_cycles = 0
        while True:
            if direction == "right":
                if self.state.stopped == True:
                    break
                GPIO.output(self.Motor1A,GPIO.HIGH)
                GPIO.output(self.Motor1B,GPIO.LOW)
                GPIO.output(self.Motor1E,GPIO.HIGH)
                sleep(sleep_speed)
                GPIO.output(self.Motor1E,GPIO.LOW)

                if self.state.stopped == True:
                    break
                GPIO.output(self.Motor2A,GPIO.LOW)
                GPIO.output(self.Motor2B,GPIO.HIGH)
                GPIO.output(self.Motor2E,GPIO.HIGH)
                sleep(sleep_speed)
                GPIO.output(self.Motor2E,GPIO.LOW)
            elif direction == "left":
                if self.state.stopped == True:
                    break
                GPIO.output(self.Motor1A,GPIO.LOW)
                GPIO.output(self.Motor1B,GPIO.HIGH)
                GPIO.output(self.Motor1E,GPIO.HIGH)
                sleep(sleep_speed)
                GPIO.output(self.Motor1E,GPIO.LOW)

                if self.state.stopped == True:
                    break
                GPIO.output(self.Motor2A,GPIO.HIGH)
                GPIO.output(self.Motor2B,GPIO.LOW)
                GPIO.output(self.Motor2E,GPIO.HIGH)
                sleep(sleep_speed)
                GPIO.output(self.Motor2E,GPIO.LOW)
            else:
                return
            num_cycles += 1
            if movement_time is not None:
                processing_time = (cv2.getTickCount() - processing_time01)/ cv2.getTickFrequency()
                if processing_time >= movement_time:
                    return (movement_time, num_cycles)

    def rotate(self, direction=None, movement_time=None):
        self.state.stopped = False
        if direction == "right":
            GPIO.output(self.Motor1A,GPIO.HIGH)
            GPIO.output(self.Motor1B,GPIO.LOW)
            GPIO.output(self.Motor1E,GPIO.HIGH)

            GPIO.output(self.Motor2A,GPIO.LOW)
            GPIO.output(self.Motor2B,GPIO.HIGH)
            GPIO.output(self.Motor2E,GPIO.HIGH)
        elif direction == "left":
            GPIO.output(self.Motor1A,GPIO.LOW)
            GPIO.output(self.Motor1B,GPIO.HIGH)
            GPIO.output(self.Motor1E,GPIO.HIGH)

            GPIO.output(self.Motor2A,GPIO.HIGH)
            GPIO.output(self.Motor2B,GPIO.LOW)
            GPIO.output(self.Motor2E,GPIO.HIGH)

        else:
            return
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def forward(self, movement_time=None):
        self.state.stopped = False
        GPIO.output(self.Motor1A,GPIO.HIGH)
        GPIO.output(self.Motor1B,GPIO.LOW)
        GPIO.output(self.Motor1E,GPIO.HIGH)

        GPIO.output(self.Motor2A,GPIO.HIGH)
        GPIO.output(self.Motor2B,GPIO.LOW)
        GPIO.output(self.Motor2E,GPIO.HIGH)

        """
        If movement_time is specified, then shut down motors after the
        amount of time, otherwise continue spinning the motors ad infinitum
        """
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def backward(self, movement_time=None):
        self.state.stopped = False
        GPIO.output(self.Motor1A,GPIO.LOW)
        GPIO.output(self.Motor1B,GPIO.HIGH)
        GPIO.output(self.Motor1E,GPIO.HIGH)

        GPIO.output(self.Motor2A,GPIO.LOW)
        GPIO.output(self.Motor2B,GPIO.HIGH)
        GPIO.output(self.Motor2E,GPIO.HIGH)

        """
        If movement_time is specified, then shut down motors after the
        amount of time, otherwise continue spinning the motors ad infinitum
        """
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def stop(self):
        # This function will need to interrupt the previous 3 functions
        GPIO.output(self.Motor1E,GPIO.LOW)
        GPIO.output(self.Motor2E,GPIO.LOW)
        self.state.stopped = True

    def clear_gpio_motor_pins(self):
        GPIO.cleanup()
