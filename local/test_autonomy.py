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


    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_basic_autonomous_routine(self):
        output = self.a.basic_autonomous_routine()
        assert output is True


    @pytest.mark.skip(reason="Not Yet Passed.")
    def test_basic_collision_avoidance(self):
        output = self.a.test_collision_avoidance()
        assert output is not None
