import unittest
from run_plan import peak_power_plan,total_duration
class TestPlanTests(unittest.TestCase):
    def test_bidirectional_and_throttle(self):
        stages=peak_power_plan(1800,3800,20,"both",True,72);self.assertEqual([s.direction for s in stages if s.record],["upshift","downshift"]);self.assertEqual({s.throttle_pct for s in stages},{72});self.assertGreater(total_duration(stages),40)
