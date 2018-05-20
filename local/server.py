from flask import Flask
app = Flask(__name__)

class Server():

    def __init__(self):
        """
        This class is used to control the web server hosted on the RPi
        """
        print("Initializing Server")



    def start_webserver(self):
        # initiate connection with server
        return

    @app.route("/")
    def hello():
        return "Hello World!"
