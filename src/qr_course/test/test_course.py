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

    def test_no_flattening_raised_support(self):
        self.assertEqual(fixture(0.03)["height"], 0.03)


if __name__ == "__main__":
    unittest.main()
