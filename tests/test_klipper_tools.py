import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

def module(name):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).resolve().parents[1]/'controllers/klipper'/f'{name}.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result

tools=module('fiveaxis_tools');flow=module('fiveaxis_flow')

class ToolTests(unittest.TestCase):
    def test_extruder_switch_restores_native_checks(self):
        a=SimpleNamespace(check_move=lambda m,i:'a',filament_area=2)
        b=SimpleNamespace(check_move=lambda m,i:'b',filament_area=2)
        instance=flow.FiveAxisFlow.__new__(flow.FiveAxisFlow)
        instance.native=None;instance.enabled=False
        instance.printer=SimpleNamespace(lookup_object=lambda name:{'extruder':a,'extruder1':b}[name],command_error=ValueError)
        instance.select_extruder('extruder');self.assertEqual(a.check_move,instance.check_move)
        instance.select_extruder('extruder1');self.assertEqual(a.check_move(None,0),'a')
        self.assertEqual(b.check_move,instance.check_move);instance.disable(None)
        self.assertEqual(b.check_move(None,0),'b')

    def test_sensor_guards_fail_closed(self):
        instance=tools.FiveAxisTools.__new__(tools.FiveAxisTools)
        sensor=SimpleNamespace(get_status=lambda time:{'filament_detected':True})
        instance.printer=SimpleNamespace(lookup_object=lambda name,default=None:sensor,get_reactor=lambda:SimpleNamespace(monotonic=lambda:10))
        command=SimpleNamespace(get=lambda key:'latch',get_int=lambda key,**kw:1,error=ValueError)
        instance.confirm(command);self.assertFalse(instance.detached)
        command.get_int=lambda key,**kw:0
        with self.assertRaisesRegex(ValueError,'guard failed'):instance.confirm(command)
        instance.printer.lookup_object=lambda *args:None
        with self.assertRaisesRegex(ValueError,'unavailable'):instance.confirm(command)

    def test_selector_only_writes_configured_pin(self):
        instance=tools.FiveAxisTools.__new__(tools.FiveAxisTools)
        instance.selectors={'0':{'pin':'selector','value':.5}};commands=[]
        instance.gcode=SimpleNamespace(run_script_from_command=commands.append)
        command=SimpleNamespace(get=lambda key:'0',error=ValueError)
        instance.select(command);self.assertEqual(commands,['SET_PIN PIN=selector VALUE=0.5'])
        command.get=lambda key:'unknown'
        with self.assertRaisesRegex(ValueError,'not configured'):instance.select(command)

if __name__=='__main__':unittest.main()
