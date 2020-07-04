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

      self.AN2 = 13
      self.AN1 = 12
      self.DIG2 = 24
      self.DIG1 = 26
      GPIO.setup(self.AN2, GPIO.OUT)
      GPIO.setup(self.AN1, GPIO.OUT)
      GPIO.setup(self.DIG2, GPIO.OUT)
      GPIO.setup(self.DIG1, GPIO.OUT)
      self.p1 = GPIO.PWM(self.AN1, 100)
      self.p2 = GPIO.PWM(self.AN2, 100)

      # This will either be GPIO.HIGH or GPIO.LOW
      # of the most recent inbound movement signal.
      self.signal = GPIO.LOW
      # Set the motor to the most recent inbound
      # movement signal
      self.motor = self.DIG1

      # The class level variable to show the most
      # recent inbound movement signal
      self.speed_percentage = 0

    def motor_controller_movement_cycle(self):
      sleep(1)
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

    def rotate(self, direction=None, movement_time=500):
        self.state.stopped = False
        if direction == "right":
          GPIO.output(self.DIG1, GPIO.LOW)
          GPIO.output(self.DIG2, GPIO.HIGH)
          self.p1.start(10)
          self.p2.start(10)
          time.sleep(movement_time)
        elif direction == "left":
          GPIO.output(self.DIG1, GPIO.HIGH)
          GPIO.output(self.DIG2, GPIO.LOW)
          self.p1.start(10)
          self.p2.start(10)
          time.sleep(movement_time)
        else:
            return
        if movement_time != None:
          sleep(movement_time)
          self.stop()

    def move_robot(self, axis_name, axis_value):
      if axis_name == "Axis 1":
        motor_position = 'left motor'
        self.motor = self.DIG1
      elif axis_name == "Axis 0":
        motor_position = 'right motor'
        self.motor = self.DIG2
      else:
        print("A New Axis has been moved.")

      # The deadzone threshold is the area where the motors are off.
      # This is useful due to the difficulty of achieving 0 in a continuum.
      deadzone_threshold = 0.50
      if axis_value > deadzone_threshold:
        self.signal = GPIO.LOW
        # speed_percentage goed from 0 to 100 while
        # axis_value goes from 0 - 1 and -1 to 0
        self.speed_percentage = (axis_value-deadzone_threshold)*100
      elif axis_value < (deadzone_threshold*-1):
        self.signal = GPIO.HIGH
        self.speed_percentage = (axis_value-deadzone_threshold)*(100*-1) - deadzone_threshold
      else:
        # Between -1*0.10 and 0.10
        # Stop all motors
        self.speed_percentage = 0

      # Update the PWM signal to the dc motor controllwer which will in turn
      # update the dc motors
      GPIO.output(self.motor, self.signal)
      if motor_position == 'left motor':
        self.p1.start(self.speed_percentage)
      if motor_position == 'right motor':
        self.p2.start(self.speed_percentage)

    def forward(self, movement_time=500, speed_percentage=10):
      '''
      Args:
        movement_time: the time in ms to travel at the given speed. If
          movement_time is specified, then shut down motors after the amount
          of time, otherwise continue spinning the motors ad infinitum
        speed: percentage of maximum speed (values 0 - 100)
      '''
      GPIO.output(self.DIG1, GPIO.LOW)
      GPIO.output(self.DIG2, GPIO.LOW)
      self.p1.start(speed_percentage)
      self.p2.start(speed_percentage)
      time.sleep(movement_time)

    def backward(self, movement_time=500, speed_percentage=10):
      '''
      Args:
        movement_time: the time in ms to travel at the given speed. If
          movement_time is specified, then shut down motors after the amount
          of time, otherwise continue spinning the motors ad infinitum
        speed: percentage of maximum speed (values 0 - 100)
      '''
      GPIO.output(self.DIG1, GPIO.HIGH)
      GPIO.output(self.DIG2, GPIO.HIGH)
      self.p1.start(speed_percentage)
      self.p2.start(speed_percentage)
      time.sleep(movement_time)

    def stop_motors(self):
      self.p1.start(0)
      self.p2.start(0)

    def stop(self):
      self.stop_motors()
      state.stopped = True

    def clear_gpio_motor_pins(self):
      return GPIO.cleanup()
