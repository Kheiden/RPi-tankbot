import autonomy
import unittest

class TestAutonomy(unittest.TestCase):

  def setUp(self):
    """ DEPRICATED. Please move this code to server/autonomy. """
    self.a = autonomy.Autonomy()

  def tearDown(self):
    pass

  @unittest.skip(reason="Unable to test.")
  def test_autonomous_routine_basic(self):
    output = self.a.autonomous_routine_basic()
    self.assertTrue(output)

  @unittest.skip(reason="Unable to test.")
  def test_collision_avoidance_basic(self):
    output = self.a.collision_avoidance_basic()
    self.assertNotEqual(output, None)
