import RPi.GPIO as GPIO
#import pigpio
from time import sleep
import threading
import state
import time
import cv2

class Movement():

    def __init__(self):
        # Inport the robot's state
        self.state = state.State()
        # Use the pin numbering from the BOARD
        GPIO.setmode(GPIO.BOARD)
        #

        self.left_pin = 37
        self.right_pin = 35

        GPIO.setup(self.left_pin,GPIO.OUT)
        GPIO.setup(self.right_pin,GPIO.OUT)


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

    def forward_slow_thread(self, movement_time, sleep_time):
        thread = threading.Thread(group=None, target=self.forward_slow, name="Thread_num0", args=(movement_time, sleep_time))
        thread.start()


    def forward_slow(self, movement_time, sleep_time):
        while True:
            if self.state.stopped == True:
                break
            self.forward(movement_time=movement_time)
            time.sleep(movement_time)


    def forward(self, movement_time=None, speed=180):
        self.state.stopped = False

        motor_left=GPIO.PWM(self.left_pin,50)
        motor_right=GPIO.PWM(self.right_pin,50)

        time.sleep(0.1)

        # motor_left.start(2.5)
        # motor_right.start(2.5)
        #
        # time.sleep(0.25)
        #
        # motor_left.ChangeDutyCycle(8)
        # motor_right.ChangeDutyCycle(8)

        motor_left.start(8)
        motor_right.start(8)

        time.sleep(1)

        motor_left.stop()
        motor_right.stop()

        """
        If movement_time is specified, then shut down motors after the
        amount of time, otherwise continue spinning the motors ad infinitum
        """
        if movement_time != None:
            sleep(movement_time)
            self.stop_motors()

    def backward(self, movement_time=None):
        self.state.stopped = False
        GPIO.output(self.Motor1A,GPIO.LOW)
        GPIO.output(self.Motor1B,GPIO.HIGH)
        GPIO.output(self.Motor1E,GPIO.HIGH)

        GPIO.output(self.Motor2A,GPIO.LOW)
        GPIO.output(self.Motor2B,GPIO.HIGH)  # not ok
        GPIO.output(self.Motor2E,GPIO.HIGH)

        """
        If movement_time is specified, then shut down motors after the
        amount of time, otherwise continue spinning the motors ad infinitum
        """
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def stop_motors(self):
        motor_left=GPIO.PWM(self.left_pin,50)
        motor_right=GPIO.PWM(self.right_pin,50)

        time.sleep(0.1)

        motor_left.start(2.5)
        motor_right.start(2.5)

        time.sleep(1)

        motor_left.stop()
        motor_right.stop()

    def stop(self):
        # This function will need to interrupt the previous 3 functions
        self.stop_motors()
        state.stopped = True

    def clear_gpio_motor_pins(self):
      # Not needed for pigpio tests
        return GPIO.cleanup()
