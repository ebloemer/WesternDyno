from pathlib import Path
import tempfile,unittest
from track_profile import CharacterizationProfile,TrackProfile

class ProfileTests(unittest.TestCase):
    def test_track_metadata_and_interpolation(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"track.csv";path.write_text("# profile_name=Test\n# actuator=pressure\n# target=engine_rpm\n# units=metric\ntime_s,target,throttle_pct\n0,1000,10\n2,3000,50\n",encoding="utf-8")
            p=TrackProfile.from_csv(path);point=p.target_at(1);self.assertEqual((p.actuator,p.target_type),("pressure","engine_rpm"));self.assertEqual((point.target,point.throttle_pct),(2000,30))
    def test_characterization_expands_repeats(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"cvt.csv";path.write_text("# actuator=pressure\n# target=torque\n# units=metric\nstage,target,transition_s,hold_s,throttle_pct,group_repeats\nPulse,50,.1,2,70,3\n",encoding="utf-8")
            self.assertEqual(len(CharacterizationProfile.from_csv(path).stages),3)
    def test_repeat_group_replays_the_whole_sequence(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"pulse.csv";path.write_text("# actuator=pressure\n# target=torque\n# units=metric\nstage,target,transition_s,hold_s,throttle_pct,group,group_repeats\nLoad,50,.1,1,70,pulse,2\nRelease,5,.1,1,70,pulse,2\n",encoding="utf-8")
            p=CharacterizationProfile.from_csv(path);self.assertEqual([s.stage for s in p.stages],["Load","Release","Load","Release"])
    def test_characterization_rejects_flow(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/"bad.csv";path.write_text("# actuator=flow\n# target=torque\n# units=metric\nstage,target,transition_s,hold_s,throttle_pct\nBad,1,1,1,20\n",encoding="utf-8")
            with self.assertRaises(ValueError):CharacterizationProfile.from_csv(path)
