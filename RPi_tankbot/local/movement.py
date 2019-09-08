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
        #GPIO.setmode(GPIO.BOARD)
        # Motor1 is the right motor
        #self.Motor1A = 33
        #self.Motor1B = 35
        #self.Motor1E = 37

        # Motor2 is the left motor
        #self.Motor2A = 36
        #self.Motor2B = 38
        #self.Motor2E = 40

        #GPIO.setup(self.Motor1A,GPIO.OUT)
        #GPIO.setup(self.Motor1B,GPIO.OUT)
        #GPIO.setup(self.Motor1E,GPIO.OUT)

        #GPIO.setup(self.Motor2A,GPIO.OUT)
        #GPIO.setup(self.Motor2B,GPIO.OUT)
        #GPIO.setup(self.Motor2E,GPIO.OUT)

        # Use the pin numbering from BROADCOM
        self.Motor1A = 13
        self.Motor1B = 19
        self.Motor1E = 26

        self.Motor2A = 16
        self.Motor2B = 20
        self.Motor2E = 21

    def run_through_gpios(self):
      print("Debugging GPIO Pins")
      self.pi = pigpio.pi()
      frequency = 16000
      # Must be 25 to 40000
      pwm_range = 25
      self.pi.set_PWM_range(self.Motor1A, pwm_range)
      self.pi.set_PWM_range(self.Motor1B, pwm_range)
      self.pi.set_PWM_range(self.Motor1E, pwm_range)

      self.pi.set_PWM_frequency(self.Motor1A, frequency)
      self.pi.set_PWM_frequency(self.Motor1B, frequency)
      self.pi.set_PWM_frequency(self.Motor1E, frequency)
      # The below list counts up from `start_number` to `stop_number` then down
      # to `start_number` again. It steps in increments of `increment`.
      start_number = 0
      stop_number = 25
      increment = 1
      create_list = [i for i in range(start_number, stop_number, increment)] + [i for i in range(stop_number, start_number-1, -increment)]
      for dutycycle in create_list:
        self.pi.set_PWM_dutycycle(self.Motor1A, dutycycle)
        self.pi.set_PWM_dutycycle(self.Motor1B, dutycycle)
        self.pi.set_PWM_dutycycle(self.Motor1E, dutycycle)
        print("sleeping for 1 second")
        time.sleep(1)
        print("Now trying dutycycle {}/{} and frequency `{}`".format(dutycycle,
          pwm_range,
          frequency))

      print("Any Change?")
      return True

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


    def forward(self, movement_time=None):
        self.state.stopped = False
        GPIO.output(self.Motor1A,GPIO.HIGH)
        GPIO.output(self.Motor1B,GPIO.LOW)
        GPIO.output(self.Motor1E,GPIO.HIGH)

        GPIO.output(self.Motor2A,GPIO.HIGH) # ok
        GPIO.output(self.Motor2B,GPIO.LOW) # not ok
        GPIO.output(self.Motor2E,GPIO.HIGH) #ok

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
        GPIO.output(self.Motor1E,GPIO.LOW)
        GPIO.output(self.Motor2E,GPIO.LOW)

    def stop(self):
        # This function will need to interrupt the previous 3 functions
        self.stop_motors()
        self.state.stopped = True

    def clear_gpio_motor_pins(self):
      # Not needed for pigpio tests
        return GPIO.cleanup()
