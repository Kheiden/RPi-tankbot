import pytest

import server
from flask import url_for

class TestServer():

    @classmethod
    def setup_class(self):
        self.s = server.Server()

    @classmethod
    def teardown_class(self):
        return

    def test_webpage(self, client):
        """
        This test is used to start the webserver which is hosted on the RPi.
        It will create the web server then send a simple GET call to confirm
        that the server is up and running.
        """
        res = client.get(url_for('hello'))
        assert res.status_code == 200
        assert res.html == 'Hello World!'
