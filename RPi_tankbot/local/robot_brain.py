import requests

class RobotBrain ():

    def __init__(self):
        """ This class is used to handle the requests made to an external server

        The RPi has limited processing power, so sending high level processing
        to an external server allows for the lower level processes to execute on
        the RPi in a reasonable amount of time.
        """
        self.server_ip_address = "192.168.1.10"
        self.server_port = "7842"

    def server_alive(self):
        """ This function is used to determine if the server is online or not.
        """
        result = requests.get("http://{ip}:{port}/v1/serveronline".format(
            ip=self.server_ip_address,
            port=self.server_port
        ))
        return result

    def disparity_map(self, payload):
        """

        This function is used to calculate the disparity map of a left and
        right image pair by sending it to a server for the calculations.
        """
        disparity_map = requests.post("http://{ip}:{port}/v1/disparitymap".format(
            ip=self.server_ip_address,
            port=self.server_port
            ), data=payload)

        return disparity_map
