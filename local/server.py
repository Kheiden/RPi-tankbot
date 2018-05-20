from flask import Flask, render_template

import movement

class Server():

    def __init__(self):
        """
        This class is used to control the web server hosted on the RPi
        """
        self.m = movement.Movement()


    def start_webserver(self):
        print("Initializing Server")
        app = Flask(__name__)

        @app.route("/")
        def hello():
            return render_template('main.html')

        @app.route("/stop")
        def stop():
            self.m.stop()
            return 200

        @app.route("/forwards")
        def go_forwards():
            self.m.forward()
            return 200

        @app.route("/backwards")
        def go_forwards():
            self.m.backward()
            return 200

        @app.route("/turn_right")
        def go_forwards():
            self.m.rotate(direction="right")
            return 200

        @app.route("/turn_left")
        def go_forwards():
            self.m.rotate(direction="left")
            return 200
            
        return app

if __name__ == '__main__':
    s = Server()
    app = s.start_webserver()
    app.debug=True
    app.run(port=5000)
