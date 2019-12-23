import RPi.GPIO as GPIO
import time
import random

from pandas import DataFrame

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

coil_A_1_pin = 11 # orange (Actual color: black)
coil_A_2_pin = 12 # yellow (Actual color: green)
coil_B_1_pin = 15 # pink (Actual color: red)
coil_B_2_pin = 13 # blue (Actual color: blue)

# adjust if different
StepCount = 4
#     [BLK,GRN,RED,BLU]
#       [A,A\,B,B\]

df = DataFrame({
            'BLK': [0, 0, 1, 0],
            'GRN': [0, 0, 0, 1],
            'RED': [0, 1, 0, 0],
            'BLU': [1, 0, 0, 0],
          }
)



# GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(coil_A_1_pin, GPIO.OUT)
GPIO.setup(coil_A_2_pin, GPIO.OUT)
GPIO.setup(coil_B_1_pin, GPIO.OUT)
GPIO.setup(coil_B_2_pin, GPIO.OUT)

# GPIO.output(enable_pin, 1)

def setStep(w1, w2, w3, w4):
  """BLK,GRN,RED,BLU"""
  GPIO.output(coil_A_1_pin, int(w1))
  GPIO.output(coil_A_2_pin, int(w2))
  GPIO.output(coil_B_1_pin, int(w3))
  GPIO.output(coil_B_2_pin, int(w4))

# def forward(delay, steps):
#   for i in range(steps):
#       for j in range(StepCount):
#           setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
#           time.sleep(delay)
#
# def backwards(delay, steps):
#   for i in range(steps):
#       for j in reversed(range(StepCount)):
#           setStep(Seq[j][0], Seq[j][1], Seq[j][2], Seq[j][3])
#           time.sleep(delay)

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
      #print(df.at[j, 'BLK'], df.at[j, 'GRN'], df.at[j, 'RED'], df.at[j, 'BLU'])
      setStep(df.at[j, 'BLK'], df.at[j, 'GRN'], df.at[j, 'RED'], df.at[j, 'BLU'])
      time.sleep(delay)

if __name__ == '__main__':
    # delay = raw_input("Time Delay (ms)?")
    # steps = raw_input("How many steps forward? ")
    """
    Args:
      delay: time in ms to delay between steps
      left_steps: Maximum number of steps left
      right_steps:Maximum number of steps right
    """
    delay = 50 / 1000.0
    left_steps = 10
    right_steps = 10

    while True:
      time_sleep = random.uniform(1, 5)
      number = random.uniform(-left_steps, 10)
      steps = int(abs(number))
      if number <= 0:
        # Turn Left, or Backwards
        print('backwards %s steps' % steps)
        movement('backwards', steps, delay)
        movement('forwards', steps, delay)
        time.sleep(time_sleep)
      else:
        # Turn Right, or Forward
        print('forwards %s steps' % steps)
        movement('forward', steps, delay)
        movement('backwards', steps, delay)
        time.sleep(time_sleep)
