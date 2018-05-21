from flask import Flask, render_template

import movement

class Server():

    def __init__(self):
        """
        This class is used to control the web server hosted on the RPi
        """
        self.m = movement.Movement()

    def clear_gpio_motor_pins(self):
        self.m.clear_gpio_motor_pins()


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
        def go_backwards():
            self.m.backward()
            return 200

        @app.route("/turn_right")
        def turn_right():
            self.m.rotate(direction="right")
            return 200

        @app.route("/turn_left")
        def turn_left():
            self.m.rotate(direction="left")
            return 200

        return app

if __name__ == '__main__':
    s = Server()
    app = s.start_webserver()
    app.debug=True
    app.run(host='0.0.0.0', port=5000)
