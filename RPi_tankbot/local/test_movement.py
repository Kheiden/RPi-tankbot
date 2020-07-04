import unittest

import time
import movement

class TestMovement(unittest.TestCase):

  def setUp(self):
    self.m = movement.Movement()

  def tearDown(self):
    self.m.clear_gpio_motor_pins()

  @unittest.skip(reason="Not Yet Passed.")
  def test_red_robot_motors(self):
    """Used to figure out which GPIO pins control the PWM for the motor"""
    #output = self.m.run_through_gpios_pigpio()
    output = self.m.run_through_gpios()
    assert output

  @unittest.skip(reason="Unable to confirm integration test.")
  def test_motor_controller_movement_cycle(self):
    output = self.m.motor_controller_movement_cycle()

  #@unittest.skip(reason="Not Yet Passed.")
  def test_move_robot(self):
      """move forward then stop
      ."""
      output = self.m.motor_precheck()
      print(output)
      self.m.forward(movement_time=5, speed_percentage=8)
      self.m.stop()
      self.m.forward(movement_time=5, speed_percentage=8, motors='left')
      self.m.stop()
      self.m.forward(movement_time=5, speed_percentage=8, motors='right')
      self.m.stop()

  @unittest.skip(reason="Not yet passed.")
  def test_camera_rotation(self):
      for i in range(-90, 91, 10):
          print("Moving camera to {}".format(i))
          self.m.move_camera(i, i)
          time.sleep(0.5)
          print("Stopping servos...")
          self.m.stop_servos()
          time.sleep(0.5)
      self.assertTrue(True)

  @unittest.skip(reason="Not yet passed.")
  def test_zero_camera(self):
      self.m.move_camera(0, 0)
