import tankbot_logs

class TestCamera():

    @classmethod
    def setup_class(self):
        """ setup any state specific to the execution of the given class (which
        usually contains tests).
        """
        self.log = tankbot_logs.RobotLog()

    @classmethod
    def teardown_class(self):
        pass

    @pytest.mark.skip(reason="Test Failing")
    def test_log_example(self):
        msg = "TESTING..."
        result = self.log.debug(msg)
        assert result == True
