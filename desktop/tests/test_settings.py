import json
from pathlib import Path
import tempfile
import unittest

from settings import SettingsStore, SetupProfile


class SettingsTests(unittest.TestCase):
    def test_edit_uses_stable_id_and_delete_is_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            store = SettingsStore(Path(directory) / "settings.json")
            created = store.create_profile(SetupProfile(profile_id="new-id", name="CVT A", configuration="cvt"))
            updated = store.update_profile(created.profile_id, SetupProfile(profile_id="ignored", name="CVT A revised", configuration="cvt"))
            self.assertEqual(updated.profile_id, created.profile_id)
            self.assertEqual(len(store.profiles), 2)
            store.delete_profile(created.profile_id)
            self.assertEqual(len(store.profiles), 1)

    def test_schema_one_name_selection_migrates_to_id(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text(json.dumps({
                "schema_version": 1,
                "preferences": {"last_setup": "Old CVT"},
                "profiles": [{
                    "name": "Old CVT", "configuration": "cvt",
                    "engine_to_pump_ratio": 0.7, "cvt_output_to_pump_ratio": 0.7,
                    "final_drive_ratio": 8.5, "tire_diameter_in": 23.0,
                    "cvt_primary": "", "cvt_secondary": "", "cvt_belt": "", "notes": ""
                }]
            }), encoding="utf-8")
            store = SettingsStore(path)
            self.assertEqual(store.preferences.last_setup_id, store.profiles[0].profile_id)
            self.assertTrue(store.profiles[0].profile_id)
