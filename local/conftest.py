import pytest

from server import start_webserver


@pytest.fixture
def app():
    app = start_webserver()
    app.debug = True
    return app
