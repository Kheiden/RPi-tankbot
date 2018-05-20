from flask import Flask, jsonify


class Server():

    def __init__(self):
        """
        This class is used to control the web server hosted on the RPi
        """
        print("Initializing Server")

    def start_webserver(self):
        # initiate connection with server
        app = Flask(__name__)

        @app.route("/")
        def hello():
            return jsonify(hello='world!')

        return app
