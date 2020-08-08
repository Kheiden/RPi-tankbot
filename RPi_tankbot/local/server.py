from flask import Flask, render_template, Response
from flask import request
from flask import make_response

import movement
import camera

class Server():

  def __init__(self):
      """
      This class is used to control the web server hosted on the RPi
      """
      self.m = movement.Movement()
      self.c = camera.Camera()

  """
  The below code is used to start the Robot's webserver. It is run locally on
  the RPi itself.
  """
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

      @app.route("/v2/move", methods=['POST'])
      def move():
        axis_name = request.form['axis_name']
        axis_value = request.form['axis_value']
        controller_type = request.form['controller_type']

        output = self.m.move_robot(
          axis_name=axis_name,
          axis_value=float(axis_value),
          controller_type=controller_type)
        return make_response(output)

      @app.route("/backwards")
      def go_backwards():
          self.m.backward()
          return "ok"

      @app.route("/turn_right")
      def turn_right():
          self.m.rotate(direction="right")
          return "ok"

      @app.route("/turn_left")
      def turn_left():
          self.m.rotate(direction="left")
          return "ok"

      @app.route("/capture_cloud_point")
      def capture_cloud_point():
          self.c.capture_cloud_point()
          return "ok"

      @app.route("/disparity_map_stream")
      def disparity_map_stream():
          return Response(self.c.start_disparity_map(),
              mimetype='multipart/x-mixed-replace; boundary=frame')

      @app.route("/take_stereo_photo")
      def take_stereo_photo():
          return Response(self.c.take_stereo_photo_yield(),
              mimetype='multipart/x-mixed-replace; boundary=frame')

      @app.route("/take_photo_remote")
      def take_stereo_photo_remote():
        print(request.headers
        return Response(self.c.take_photo(),
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
  print("Starting up server...")
  try:
    app.run(host='0.0.0.0', port=5000)
  except KeyboardInterrupt:
    print("Interrupt Signal Sent.")
  finally:
    print("Shutting down server...")
    s.clear_gpio_motor_pins()
