import RPi.GPIO as GPIO
import time
import random

from pandas import DataFrame

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

coil_A_1_pin = 40 # orange (Actual color: black)
coil_A_2_pin = 36 # yellow (Actual color: green)
coil_B_1_pin = 38 # pink (Actual color: red)
coil_B_2_pin = 32 # blue (Actual color: blue)

# adjust if different
StepCount = 4
#     [BLK,GRN,RED,BLU]
#       [A,A\,B,B\]

df = DataFrame({
            'BLK': [0, 0, 0, 1],
            'GRN': [0, 1, 0, 0],
            'RED': [0, 0, 1, 0],
            'BLU': [1, 0, 0, 0],
          }
)


GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)

def setStep(w1, w2, w3, w4):
  """BLK,GRN,RED,BLU"""
  GPIO.output(coil_A_1_pin, int(w1))
  GPIO.output(coil_A_2_pin, int(w2))
  GPIO.output(coil_B_1_pin, int(w3))
  GPIO.output(coil_B_2_pin, int(w4))


def movement(direction, steps, delay):
  """
  Args:
    direction: 'forward' or 'backwards'
  """
  if direction is 'forward':
    step_series = range(StepCount)
  else:
    step_series = reversed(range(StepCount))

  for i in range(steps):
    for j in step_series:
      # setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
      print(df.at[j, 'BLK'], df.at[j, 'GRN'], df.at[j, 'RED'], df.at[j, 'BLU'])
      setStep(df.at[j, 'BLK'], df.at[j, 'GRN'], df.at[j, 'RED'], df.at[j, 'BLU'])
      time.sleep(delay)

if __name__ == '__main__':
    # delay = raw_input("Time Delay (ms)?")
    # steps = raw_input("How many steps forward? ")
    """
    Args:
      delay: time in ms to delay between steps. 2 ms is lowest torque and
        highest speed
      left_steps: Maximum number of steps left
      right_steps:Maximum number of steps right
    """
    delay = 2 / 1000.0
    steps = 100
    time_sleep = 1

    while True:
      movement('forward', steps, delay)
      time.sleep(time_sleep)
