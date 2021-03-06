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
            return "ok"

        @app.route("/forwards")
        def go_forwards():
            self.m.forward()
            return "ok"

        @app.route("/backwards")
        def go_backwards():
            self.m.backward()
            return "ok"

        @app.route("/turn_right")
        def turn_right():
            #self.m.rotate(direction="right")
            self.m.rotate_on_carpet(direction="right", sleep_speed=0.33)
            return "ok"

        @app.route("/turn_left")
        def turn_left():
            #self.m.rotate(direction="left")
            self.m.rotate_on_carpet(direction="left", sleep_speed=0.33)
            return "ok"

        @app.route("/disparity_map_stream")
        def disparity_map_stream():
            return Response(self.c.start_disparity_map(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route("/left_camera_stream")
        def left_camera_stream():
            return Response(self.c.start_left_camera(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route("/right_camera_stream")
        def right_camera_stream():
            return Response(self.c.start_right_camera(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        @app.route("/left_and_right_camera_stream")
        def left_and_right_camera_stream():
            return Response(self.c.start_left_and_right_cameras(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

        return app

if __name__ == '__main__':
    s = Server()
    app = s.start_webserver()
    app.debug=True
    app.run(host='0.0.0.0', port=5000)
    s.clear_gpio_motor_pins()
