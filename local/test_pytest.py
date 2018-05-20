
class TestPytest:

    def hi(self):
        return "hi"

    def test_hi(self):
        assert self.hi() == "hi"

    def test_hi_fail(self):
        assert self.hi() == "hii"
