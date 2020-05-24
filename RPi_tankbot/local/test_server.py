import unittest

import server
import time
from flask import url_for

class TestServer(unittest.TestCase):

  def setUp(self):
    self.s = server.Server()

  def tearDown(self):
    self.s.clear_gpio_motor_pins()

  @unittest.skip(reason="Skipping.")
  def test_basic_webpage(self, client):
    """
    This test is used to start the webserver which is hosted on the RPi.
    It will create the web server then send a simple GET call to confirm
    that the server is up and running.
    """
    res = client.get(url_for('hello'))
    assert res.status_code == 200
    assert res.json == {'hello': 'world!'}

  @unittest.skip(reason="Skipping.")
  def test_left_video_stream(self, client):
    res = client.get(url_for('left_camera_stream'))
    assert res.status_code == 200
    assert res
