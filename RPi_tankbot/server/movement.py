import requests

class Movement():

    def __init__(self):
        """ This class is used to control movement via API calls to the server
        that is running on the RPi.
        """
        self.host_ip = '192.168.1.16'
        self.host_port = '5000'

    def stop(self):
        requests.get("http://{ip}:{port}/stop".format(
            ip=self.host_ip,
            port=self.port))

    def forwards(self):
        requests.get("http://{ip}:{port}/forwards".format(
            ip=self.host_ip,
            port=self.port))

    def backwards(self):
        requests.get("http://{ip}:{port}/backwards".format(
            ip=self.host_ip,
            port=self.port))

    def turn_right(self):
        requests.get("http://{ip}:{port}/turn_right".format(
            ip=self.host_ip,
            port=self.port))

    def turn_left(self):
        requests.get("http://{ip}:{port}/turn_left".format(
            ip=self.host_ip,
            port=self.port))
