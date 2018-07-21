class State():

    def __init__(self):
        # All variables are set to None since they are unknown. The robot will update
        # them as needed.
        self.stopped = None
        self.rotating = None
        self.patrolling = None
        self.taking_360_cloud_point = None
