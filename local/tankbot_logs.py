import os.path
import logging


class RobotLog:
    """
    Used to debug issues with the Robot platform
    """
    def __init__(self):
        log_path = "/home/pi/RPi-tankbot/logs/robot.log"
        self.filename = os.path.abspath(log_path)
        logging.basicConfig(
            filename=self.filename,
            level=logging.DEBUG,
            format='%(asctime)s %(message)s')

    def debug(self, message):
        print("Saving '{}' to {}".format(message, self.filename))

        logging.debug(message)
        return True

class ServerLog:
    """
    Used to log the traffic and usage of the Tankbot Web Controller
    """

    def __init__(self):
        log_path = "/home/pi/RPi-tankbot/logs/server.log"
        logging.basicConfig(
            filename=os.path.abspath(log_path),
            level=logging.DEBUG,
            format='%(asctime)s %(message)s')

    def debug(self, message):
        logging.debug(message)
        return True
