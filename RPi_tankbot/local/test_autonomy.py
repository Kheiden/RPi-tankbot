import autonomy
import pytest

class TestAutonomy():

    @classmethod
    def setup_class(self):
        self.a = autonomy.Autonomy()

    @classmethod
    def teardown_class(self):
        pass


    @pytest.mark.skip(reason="Passed.")
    def test_autonomous_routine_basic(self):
        output = self.a.autonomous_routine_basic()
        assert output is True


    @pytest.mark.skip(reason="Passed.")
    def test_collision_avoidance_basic(self):
        output = self.a.collision_avoidance_basic()
        assert output is not None
