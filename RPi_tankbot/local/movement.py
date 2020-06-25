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

        AN2 = 13
        AN1 = 12
        DIG2 = 24
        DIG1 = 26
        GPIO.setup(AN2, GPIO.OUT)
        GPIO.setup(AN1, GPIO.OUT)
        GPIO.setup(DIG2, GPIO.OUT)
        GPIO.setup(DIG1, GPIO.OUT)

    def motor_controller_movement_cycle(self):
      sleep(1)
      p1 = GPIO.PWM(AN1, 100)
      p2 = GPIO.PWM(AN2, 100)

      print("Forward")
      self.forward()
      print("Backward")
      self.backward()
      print("Left")
      self.rotate(direction="left")
      print("Right")
      self.rotate(direction="right")
      print("STOP")

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
          GPIO.output(DIG1, GPIO.LOW)
          GPIO.output(DIG2, GPIO.HIGH)
          p1.start(10)
          p2.start(10)
        elif direction == "left":
          GPIO.output(DIG1, GPIO.HIGH)
          GPIO.output(DIG2, GPIO.LOW)
          p1.start(10)
          p2.start(10)
        else:
            return
        if movement_time != None:
          sleep(movement_time)
          self.stop()

    def forward(self, movement_time=None, speed_percentage=10):
      '''
      Args:
        movement_time: the time in ms to travel at the given speed. If
          movement_time is specified, then shut down motors after the amount
          of time, otherwise continue spinning the motors ad infinitum
        speed: percentage of maximum speed (values 0 - 100)
      '''
      GPIO.output(DIG1, GPIO.LOW)
      GPIO.output(DIG2, GPIO.LOW)
      p1.start(speed_percentage)
      p2.start(speed_percentage)

    def backward(self, movement_time=None, speed_percentage=10):
      '''
      Args:
        movement_time: the time in ms to travel at the given speed. If
          movement_time is specified, then shut down motors after the amount
          of time, otherwise continue spinning the motors ad infinitum
        speed: percentage of maximum speed (values 0 - 100)
      '''
      GPIO.output(DIG1, GPIO.HIGH)
      GPIO.output(DIG2, GPIO.HIGH)
      p1.start(speed_percentage)
      p2.start(speed_percentage)

    def stop_motors(self):
      GPIO.output(DIG1, GPIO.LOW)
      GPIO.output(DIG2, GPIO.LOW)
      p1.start(0)
      p2.start(0)

    def stop(self):
      self.stop_motors()
      state.stopped = True

    def clear_gpio_motor_pins(self):
      return GPIO.cleanup()
