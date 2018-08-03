import autonomy
import pytest

class TestAutonomy():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """
        self.a = autonomy.Autonomy()

    @classmethod
    def teardown_class(self):
        """ teardown any state that was previously setup with a call to
        setup_class.
        """
        pass


    @pytest.mark.skip(reason="Passed.")
    def test_autonomous_routine_basic(self):
        output = self.a.autonomous_routine_basic()
        assert output is True


    @pytest.mark.skip(reason="Passed.")
    def test_collision_avoidance_basic(self):
        output = self.a.collision_avoidance_basic()
        assert output is not None
