import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace

path = Path(__file__).resolve().parents[1] / 'controllers/klipper/fiveaxis_flow.py'
spec = importlib.util.spec_from_file_location('flow', path)
flow = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flow)

class FlowTests(unittest.TestCase):
    def test_cold_extrusion_guard(self):
        instance=flow.FiveAxisFlow.__new__(flow.FiveAxisFlow)
        instance.enabled=True
        instance.extruder=SimpleNamespace(heater=SimpleNamespace(can_extrude=False))
        instance.printer=SimpleNamespace(command_error=ValueError)
        move=SimpleNamespace(axes_d=[1,0,0,.1,0,0])
        with self.assertRaisesRegex(ValueError,'Extrude below minimum temp'):
            instance.check_move(move,3)

    def test_retraction_retains_native_checks(self):
        instance=flow.FiveAxisFlow.__new__(flow.FiveAxisFlow)
        instance.enabled=True
        instance.native=lambda move,index: (move.axes_d[index],index)
        move=SimpleNamespace(axes_d=[0,0,0,-1,0,0])
        self.assertEqual(instance.check_move(move,3),(-1,3))
    def test_rotary_only_distance(self):
        a = [10, 0, 0, 0, 0, 0]
        b = [10, 0, 0, .1, 0, 90]
        self.assertAlmostEqual(flow.deposition_length(a, b, 4, 5, (0, 0, 0), 'BED'), 5*3.141592653589793, places=4)

    def test_stationary_priming(self):
        a = [10, 0, 0, 0, 0, 0]
        b = [10, 0, 0, 1, 0, 0]
        self.assertEqual(flow.deposition_length(a, b, 4, 5, (0, 0, 0), 'BED'), 0)

    def test_head_tcp_motion(self):
        a = [0, 0, 0, 0, 0, 0]
        b = [3, 4, 0, 1, 45, 45]
        self.assertAlmostEqual(flow.deposition_length(a, b, 4, 5, (0, 0, 0), 'HEAD'), 5)
    def test_head_pivot_compensation(self):
        a=[0,0,10,0,0,0];b=[15,0,-5,0,0,90]
        point=flow.part_point(b,4,5,(0,0,0),'HEAD',(0,0,0),(0,0,30),(0,0,15))
        for actual,expected in zip(point,(0,0,10)):self.assertAlmostEqual(actual,expected)
        self.assertGreater(flow.deposition_length(a,b,4,5,(0,0,0),'HEAD',(0,0,0),(0,0,30),(0,0,15)),8)

    def test_bed_tcp_offset(self):
        point=flow.part_point([8,0,0,0,0,90],4,5,(0,0,0),'BED',(2,0,0))
        self.assertAlmostEqual(point[0],0);self.assertAlmostEqual(point[1],-10)

    def test_pivot(self):
        point = flow.part_point([11, 2, 3, 0, 0, 90], 4, 5, (1, 2, 3), 'BED')
        self.assertAlmostEqual(point[0], 1)
        self.assertAlmostEqual(point[1], -8)

if __name__ == '__main__':
    unittest.main()
