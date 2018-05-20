import pytest

import movement

class TestMovement():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """

        self.m = movement.Movement()

    @classmethod
    def teardown_class(self):
        """ teardown any state that was previously setup with a call to
        setup_class.
        """
        return

    def test_move_robot(self):
        """move forward, turn, then move forward again."""
        self.m.forward(movement_time=3)
        self.m.backward(movement_time=3)
        self.m.rotate(direction="right", movement_time=3)
        self.m.rotate(direction="left", movement_time=3)
