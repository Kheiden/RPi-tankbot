import unittest

import movement

class TestMovement(unittest.TestCase):

  def setUp(self):
    self.m = movement.Movement()

  def tearDown(self):
    self.m.clear_gpio_motor_pins()

  @unittest.skip(reason="Not Yet Passed.")
  def test_move_robot(self):
    axis_name = "Axis 0"
    axis_value = 0.45
    controller_type = 
    self.m.move_robot(axis_name, axis_value, controller_type)
