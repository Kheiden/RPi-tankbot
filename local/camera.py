import RPi.GPIO as GPIO
import time

class Camera():

    def __init__(self):
        self.servo_axis_x_pin = 11
        self.servo_axis_y_pin = 13

        x_axis_degrees = 0
        y_axis_degrees = 0
        # x_axis
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11,GPIO.OUT)
        self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
        self.pwm_x.start(1/18*(x_axis_degrees+90)+2)
        #self.pwm_x.ChangeDutyCycle(3.5)

        # y_axis
        GPIO.setup(13,GPIO.OUT)
        self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
        self.pwm_y.start(1/18*(y_axis_degrees+90)+2)
        #self.pwm_y.ChangeDutyCycle(6)
        time.sleep(1)
        self.pwm_x.stop()
        self.pwm_y.stop()
        return

    def move_camera(self, x_axis_degrees, y_axis_degrees):
        # input in degrees, output in DutyCycle
        # 0 is neutral, 45 is to the right, -45 is to the left
        self.pwm_x=GPIO.PWM(self.servo_axis_x_pin,50)
        num1 = (((float(x_axis_degrees) + 90)/18) + 2)
        print(num1, x_axis_degrees)
        self.pwm_x.start(num1)
        self.pwm_x.ChangeDutyCycle(num1)

        self.pwm_y=GPIO.PWM(self.servo_axis_y_pin,50)
        num2 = (((float(y_axis_degrees) + 90)/18) + 2)
        print(num2, y_axis_degrees)
        self.pwm_y.start(num2)
        self.pwm_y.ChangeDutyCycle(num2)
        #self.pwm_x.ChangeDutyCycle(x_axis) # neutral is 3.5
        # 0 is neutral, 45 is up, -45 is down
        #self.pwm_y.ChangeDutyCycle(y_axis)

        # after moving, stop the servo acuation
        #time.sleep(1)
        #self.pwm_x.stop()
        #self.pwm_y.stop()
        return

    def stop_servos(self):
        self.pwm_x.stop()
        self.pwm_y.stop()
        #self.pwm_x.stop()
        #self.pwm_y.stop()
