"""Explicit FiveAxisLab tool state and sensor guards; contains no machine motion.

Pickup, release, purge and all carriage motion must be present in the generated
job. Selectors map a filament slot to exactly one configured output-pin write.
"""
import json
import re


class FiveAxisTools:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.detached = False
        try:
            self.selectors = json.loads(config.get('selectors', '{}'))
            if not isinstance(self.selectors, dict):
                raise ValueError('selectors must be an object')
            for slot, route in self.selectors.items():
                if not re.fullmatch(r'[A-Za-z0-9_-]+', slot):
                    raise ValueError('Invalid slot')
                if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', route['pin']):
                    raise ValueError('Invalid output pin')
                if not 0 <= float(route['value']) <= 1:
                    raise ValueError('Selector value must be 0..1')
        except (ValueError, TypeError, KeyError) as exc:
            raise config.error('Invalid FiveAxisLab selector configuration: %s' % exc)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('FAL_TOOL_DETACH', self.detach)
        self.gcode.register_command('FAL_TOOL_CONFIRM', self.confirm)
        self.gcode.register_command('FAL_SELECT_FILAMENT', self.select)

    def detach(self, gcmd):
        flow = self.printer.lookup_object('fiveaxis_flow', None)
        if flow is not None:
            flow.disable(gcmd)
        self.detached = True

    def confirm(self, gcmd):
        name = gcmd.get('SENSOR')
        sensor = self.printer.lookup_object('filament_switch_sensor ' + name, None)
        if sensor is None:
            raise gcmd.error('Configured tool attachment sensor is unavailable: ' + name)
        expected = gcmd.get_int('PRESENT', minval=0, maxval=1)
        state = sensor.get_status(self.printer.get_reactor().monotonic())
        if 'filament_detected' not in state or bool(state['filament_detected']) != bool(expected):
            raise gcmd.error('Tool attachment sensor guard failed: ' + name)
        self.detached = not bool(expected)

    def select(self, gcmd):
        slot = gcmd.get('SLOT')
        if slot not in self.selectors:
            raise gcmd.error('Filament selector slot is not configured: ' + slot)
        route = self.selectors[slot]
        self.gcode.run_script_from_command('SET_PIN PIN=%s VALUE=%.9g' %
                                           (route['pin'], float(route['value'])))


def load_config(config):
    return FiveAxisTools(config)
