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
      # The deadzone threshold is the area where the motors are off.
      # This is useful due to the difficulty of achieving 0 in a continuum.
      # We have a deadzone in the forwards direction and in the backwards
      # direction. Each is half the size of the deadzone_threshold.
      self.deadzone_threshold = 0.40

      # This will either be GPIO.HIGH or GPIO.LOW
      # of the most recent inbound movement signal.
      self.signal = GPIO.LOW
      # Set the motor to the most recent inbound
      # movement signal
      self.motor = self.DIG1

      # The class level variable to show the most
      # recent inbound movement signal
      self.speed_percentage = 0
      # The most recent inbound movement signal
      self.direction = 'None'

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
      header_text = '--- MOTOR PRECHECK ---\n'
      return header_text + str('')

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

    def move_robot_joystick(self, axis_name, axis_value):
      self.left_motor = self.DIG2
      self.right_motor = self.DIG1
      self.signal_forwards = GPIO.LOW
      self.signal_backwards = GPIO.HIGH
      self.speed_percentage = (axis_value-self.deadzone_threshold)*100
      if axis_name == "Axis 0":
        # Axis 0 negative is translate left, positive is translate right
        pass
      elif axis_name == "Axis 1":
        # Axis 1 negative is forwards, positive is backwards
        if axis_value < self.deadzone_threshold:
          GPIO.output(self.left_motor, self.signal_forwards)
          GPIO.output(self.right_motor, self.signal_forwards)
        elif axis_value < (self.deadzone_threshold*-1):
          GPIO.output(self.left_motor, self.signal_backwards)
          GPIO.output(self.right_motor, self.signal_backwards)
        else:
          self.signal = GPIO.LOW
          self.direction = 'None'
          self.speed_percentage = 0
        self.p2.start(self.speed_percentage)
        self.p2.start(self.speed_percentage)
      elif axis_name == "Axis 3":
        # Axis 3 negative is rotate left, positive is rotate right
        pass
      # Update the PWM signal to the dc motor controllwer which will in turn
      # update the dc motors


    def move_robot_throttle(self, axis_name, axis_value):
      if axis_name == "Axis 0":
        motor_position = 'left motor'
        self.motor = self.DIG2
      elif axis_name == "Axis 1":
        motor_position = 'right motor'
        self.motor = self.DIG1
      else:
        print("A New Axis has been moved.")

      if axis_value > self.deadzone_threshold:
        # GPIO.LOW is forwards
        self.direction = 'forwards'
        self.signal = GPIO.LOW
        # speed_percentage goed from 0 to 100 while
        # axis_value goes from 0 - 1 and -1 to 0
        self.speed_percentage = (axis_value-self.deadzone_threshold)*100
      elif axis_value < (self.deadzone_threshold*-1):
        # GPIO.HIGH is backwards
        self.direction = 'backwards'
        self.signal = GPIO.HIGH
        self.speed_percentage = (axis_value+self.deadzone_threshold)*(100)*-1
      else:
        # Between -1*0.10 and 0.10
        # Stop all motors
        self.signal = GPIO.LOW
        self.direction = 'None'
        self.speed_percentage = 0

      # Update the PWM signal to the dc motor controllwer which will in turn
      # update the dc motors
      GPIO.output(self.motor, self.signal)
      output = ''
      if motor_position == 'left motor':
        self.p2.start(self.speed_percentage)
        output += "motor:{} signal:{} direction:{} speed_percentage:{}".format(
          self.motor,
          self.signal,
          self.direction,
          self.speed_percentage)
      if motor_position == 'right motor':
        self.p1.start(self.speed_percentage)
        output += "motor:{} signal:{} direction:{} speed_percentage:{}".format(
          self.motor,
          self.signal,
          self.direction,
          self.speed_percentage)

      return output

    def move_robot(self, axis_name, axis_value, controller_type):
      output = None
      if controller_type == '1':
        output = self.move_robot_joystick(axis_name,
                                          axis_value)
      elif controller_type == '2':
        output = self.move_robot_throttle(axis_name,
                                          axis_value)
      else:
        print("A new controller has been moved.")

      return output
