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
        self.m.clear_gpio_motor_pins()

    @pytest.mark.skip(reason="Passed.")
    def test_move_robot(self):
        """move forward, turn, then move forward again."""
        self.m.forward(movement_time=3)
        self.m.backward(movement_time=3)
        self.m.rotate(direction="right", movement_time=3)
        self.m.rotate(direction="left", movement_time=3)


    def test_infinite_motor_movement(self):
        """Move motors without passing movement_time variable"""
        self.m.forward()
        time.sleep(3)
        self.m.stop()
        self.m.backward()
        time.sleep(3)
        self.m.stop()
        self.m.rotate(direction="right")
        time.sleep(3)
        self.m.stop()
        self.m.rotate(direction="left")
        time.sleep(3)
        self.m.stop()
