import movement

class Client():

    def __init__(self):
        # Establish connection with server
        print("Initializing Client")
        self.m = movement.Movement()

    def connect_to_server(self):
        # initiate connection with server
        return

    def start_robot(self):
        # TODO: initate autonomous sequence
        # TODO: create a new module for different sequences
        
        print("robot started")
        print("forward")
        self.m.forward(1)
        print("backward")
        self.m.backward(1)
        print("rotate right")
        self.m.rotate(1)
        print("rotate left")
        self.m.rotate(-2)
        print("rotate back to start")
        self.m.rotate(1)

    def stop_robot(self):
        # stop motors and clear the GPIO board
        print("Robot Stopping.")
        self.m.stop()

if __name__ == '__main__':
    c = Client()
    c.start_robot()
    c.stop_robot()
