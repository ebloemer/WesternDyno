from pathlib import Path
import tempfile
import unittest

from engine_map import EnginePowerMap


class EngineMapTests(unittest.TestCase):
    def test_import_and_interpolation_without_extrapolation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "engine.csv"
            path.write_text(
                "# name,direct sweep\nengine_rpm,power_kw\n1000,2\n2000,6\n3000,8\n",
                encoding="utf-8",
            )
            power_map = EnginePowerMap.from_csv(path)
            self.assertAlmostEqual(power_map.power_at(1500), 4.0)
            self.assertIsNone(power_map.power_at(900))
            self.assertIsNone(power_map.power_at(3100))
