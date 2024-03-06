import unittest


class TestLiconic_Base(unittest.TestCase):
    pass


class TestImports(TestLiconic_Base):
    def test_liconic_driver_import(self):
        import liconic_driver

        assert liconic_driver.__version__


if __name__ == "__main__":
    unittest.main()
