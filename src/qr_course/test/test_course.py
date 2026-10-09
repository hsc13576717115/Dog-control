import sys
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from course import fixture, generate


class Course(unittest.TestCase):
    def test_invalid_heights_rejected(self):
        for h in [-0.01, 0.04, float("nan")]:
            with self.assertRaises(ValueError):
                fixture(h)

    def test_geometry_matches_published_spec(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "a.world"
            spec = generate(0.05, p)
            root = ET.parse(p)
            for pad in spec["pads"]:
                m = root.find(f"./world/model[@name=\"pad_{pad['id']}\"]")
                self.assertEqual(
                    list(map(float, m.findtext("pose").split()))[:3],
                    [*pad["center"], 0.025],
                )
                self.assertEqual(
                    list(
                        map(
                            float,
                            m.findtext("link/collision/geometry/box/size").split(),
                        )
                    ),
                    [0.2, 0.2, 0.05],
                )

    def test_platform_known_ground_stops_at_vertical_edge(self):
        spec = fixture(0.05, "platform")
        ground, platform = spec["pads"]
        self.assertAlmostEqual(
            ground["center"][0] + ground["size"][0] / 2,
            platform["center"][0] - platform["size"][0] / 2,
        )
        self.assertEqual(ground["height"], 0.0)
        self.assertEqual(platform["height"], 0.05)
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "a.world"
            generate(0.05, p, "platform")
            root = ET.parse(p)
            self.assertIsNone(root.find('./world/model[@name="pad_0"]'))
            size = root.findtext(
                './world/model[@name="pad_1"]/link/collision/geometry/box/size'
            )
            self.assertEqual(list(map(float, size.split())), [0.8, 0.8, 0.05])

    def test_no_flattening_raised_support(self):
        self.assertEqual(fixture(0.03)["height"], 0.03)


if __name__ == "__main__":
    unittest.main()
