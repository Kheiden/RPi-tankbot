from flask import Flask, render_template, Response

import movement
import camera

class Server():

    def __init__(self):
        """
        This class is used to control the web server hosted on the RPi
        """
        self.m = movement.Movement()
        self.c = camera.Camera()

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
            return

        @app.route("/forwards")
        def go_forwards():
            self.m.forward()
            return

        @app.route("/backwards")
        def go_backwards():
            self.m.backward()
            return

        @app.route("/turn_right")
        def turn_right():
            self.m.rotate(direction="right")
            return

        @app.route("/turn_left")
        def turn_left():
            self.m.rotate(direction="left")
            return

        @app.route("/left_camera_stream")
        def left_camera_stream():
            return Response(self.c.start_left_camera(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route("/right_camera_stream")
        def right_camera_stream():
            return Response(self.c.start_right_camera(),
                mimetype='multipart/x-mixed-replace; boundary=frame')


        return app

if __name__ == '__main__':
    s = Server()
    app = s.start_webserver()
    app.debug=True
    app.run(host='0.0.0.0', port=5000)
