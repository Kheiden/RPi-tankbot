import os
import logging


class RobotLog:
    """
    Used to debug issues with the Robot platform
    """
    def __init__(self):
        log_path = "../logs/dev.log"
        if not os.path.exists(os.path.dirname(log_path)):
            os.makedirs(os.path.dirname(log_path))
        logging.basicConfig(
            filename=os.path.abspath(log_path),
            level=logging.DEBUG,
            format='%(asctime)s %(message)s')

    def debug(self, message):
        logging.debug(message)
        return True

class ServerLog:
    """
    Used to log the traffic and usage of the Tankbot Web Controller
    """

    def __init__(self):
        log_path = "../logs/server.log"
        if not os.path.exists(os.path.dirname(log_path)):
            os.makedirs(os.path.dirname(log_path))
        logging.basicConfig(
            filename=os.path.abspath(log_path),
            level=logging.DEBUG,
            format='%(asctime)s %(message)s')

    def debug(self, message):
        logging.debug(message)
        return True
