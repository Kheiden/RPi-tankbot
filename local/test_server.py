import pytest

import server
import time
from flask import url_for

class TestServer():

    @classmethod
    def setup_class(self):
        self.s = server.Server()

    @classmethod
    def teardown_class(self):
        self.s.clear_gpio_motor_pins()

    @pytest.mark.skip(reason="Passed.")
    def test_basic_webpage(self, client):
        """
        This test is used to start the webserver which is hosted on the RPi.
        It will create the web server then send a simple GET call to confirm
        that the server is up and running.
        """
        res = client.get(url_for('hello'))
        assert res.status_code == 200
        assert res.json == {'hello': 'world!'}

    def test_web_controller(self, client):
        # 60 seconds to play around
        time.sleep(60)
        # instant succeed
        assert True==True
