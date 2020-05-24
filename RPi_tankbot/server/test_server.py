import requests
import unittest
from  unittest import mock

class TestServer(unittest.TestCase):

  def __init__(self):
    self.ip_address = '192.168.1.10'
    self.port = '8183'

  @unittest.skip("Skipping")
  def test_server_live(self):
    output = requests.get("{}:{}".format(self.ip_address, self.port))
    self.assertTrue(False)
