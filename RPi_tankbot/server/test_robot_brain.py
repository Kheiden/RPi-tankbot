import unittest
from  unittest import mock

class TestRobotBrain(unittest.TestCase):

  def setUp(self):
    self.robot_brain = robot_brain.RobotBrain()

  def tearDown(self):
    return

  def test_fail(self):
    self.assertTrue(self.robot_brain.fail())
