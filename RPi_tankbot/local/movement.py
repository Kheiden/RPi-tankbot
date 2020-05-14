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
        # Use the pin numbering from the BROADCOM
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def motor_controller(self):
      AN2 = 13
      AN1 = 12
      DIG2 = 24
      DIG1 = 26
      GPIO.setup(AN2, GPIO.OUT)
      GPIO.setup(AN1, GPIO.OUT)
      GPIO.setup(DIG2, GPIO.OUT)
      GPIO.setup(DIG1, GPIO.OUT)
      sleep(1)
      p1 = GPIO.PWM(AN1, 100)
      p2 = GPIO.PWM(AN2, 100)

      print("Left")
      GPIO.output(DIG1, GPIO.HIGH)
      GPIO.output(DIG2, GPIO.LOW)
      p1.start(100)
      p2.start(100)
      sleep(2)
      print("Forward")
      # set DIG1 as LOW, to control direction
      GPIO.output(DIG1, GPIO.LOW)
      # set DIG2 as LOW, to control direction
      GPIO.output(DIG2, GPIO.LOW)
      # set speed for M1 at 10%
      # set speed for M2 at 10%
      p1.start(10)
      p2.start(10)
      sleep(2)

      print( "Backward")
      GPIO.output(DIG1, GPIO.HIGH)
      GPIO.output(DIG2, GPIO.HIGH)
      p1.start(10)
      p2.start(10)
      sleep(2)

      print("Right")
      GPIO.output(DIG1, GPIO.LOW)
      GPIO.output(DIG2, GPIO.HIGH)
      p1.start(10)
      p2.start(10)
      sleep(2)

      print("STOP")
      GPIO.output(DIG1, GPIO.LOW)
      GPIO.output(DIG2, GPIO.LOW)
      p1.start(0)
      p2.start(0)
      sleep(3)

    def get_function_state(self, pin):
      '''Will return a value of:
      GPIO.IN, GPIO.OUT, GPIO.SPI, GPIO.I2C, GPIO.HARD_PWM, GPIO.SERIAL,
      GPIO.UNKNOWN'''
      return GPIO.gpio_function(pin)

    def motor_precheck(self):
      status = {pin: self.get_function_state(pin) for pin in self.all_active_pins}
      header_text = '--- MOTOR PRECHECK ---\n'
      return header_text + str(status)

    def rotate(self, direction=None, movement_time=None):
        self.state.stopped = False
        if direction == "right":
            motor_left=GPIO.PWM(self.left_pin,50)
            time.sleep(0.1)
            motor_left.start(10)
            time.sleep(1)
            motor_left.stop()
        elif direction == "left":
            motor_right=GPIO.PWM(self.right_pin,50)
            time.sleep(0.1)
            motor_right.start(10)
            time.sleep(1)
            motor_right.stop()
        else:
            return
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def flip_relay(relay_array):
      '''
      Args:
        relay_array: Takes in a list of strings denoting which relays to flip
      '''
      # # TODO: Fill this out
      return False
      # for relay in relay_array:
      #   GPIO.PWM(self.relay_pins[relay], 5000)



    def forward(self, movement_time=None, speed_percentage=100):
        '''
        Args:
          movement_time: the time in ms to travel at the given speed. If
            movement_time is specified, then shut down motors after the amount
            of time, otherwise continue spinning the motors ad infinitum
          speed: percentage of maximum speed (values 0 - 100)
        '''
        self.state.stopped = False
        duty_cycle = speed_percentage / 10
        print('Duty Cycle changed to %s' % duty_cycle)
        self.motor_left.ChangeDutyCycle(duty_cycle)
        self.motor_right.ChangeDutyCycle(duty_cycle)

        if movement_time != None:
            sleep(movement_time)
            self.stop_motors()

    def backward(self, movement_time=None, speed_percentage=100):
        '''
        Args:
          movement_time: the time in ms to travel at the given speed. If
            movement_time is specified, then shut down motors after the amount
            of time, otherwise continue spinning the motors ad infinitum
          speed: percentage of maximum speed (values 0 - 100)
        '''
        self.state.stopped = False
        output = self.flip_relay(['left', 'right'])
        if not output:
          print('ERROR: No confirmation that the relays have successfully'
            'switched')
        # Backwards is just forwards in reverse!
        forward(movement_time=movement_time, speed_percentage=speed_percentage)
        """
        If movement_time is specified, then shut down motors after the
        amount of time, otherwise continue spinning the motors ad infinitum
        """
        if movement_time != None:
            sleep(movement_time)
            self.stop()

    def stop_motors(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def stop(self):
        # This function will need to interrupt the previous 3 functions
        self.stop_motors()
        state.stopped = True

    def clear_gpio_motor_pins(self):
        return GPIO.cleanup()
