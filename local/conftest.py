import pytest

from server import Server


@pytest.fixture
def app():
    s = Server()
    app = s.start_webserver()
    app.debug = True
    return app
