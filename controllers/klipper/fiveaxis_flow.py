"""Rotary-aware deposition checks for Klipper with native GCODE_AXIS support.

Uses the actual workpiece-space TCP displacement for cross-section checks.
The underlying heater, retraction, joint-bound and motion-queue checks remain active.
Pressure advance is intentionally disabled for this research profile.
"""
import math

def part_point(position, a_index, b_index, pivot, layout, tcp=(0.,0.,0.), pivot_a=(0.,0.,0.), pivot_b=(0.,0.,0.)):
    x, y, z = position[:3]
    if layout == 'HEAD':
        a,b=math.radians(position[a_index]),math.radians(position[b_index])
        tx,ty,tz=(v-p for v,p in zip(tcp,pivot_b))
        tx,ty,tz=math.cos(b)*tx+math.sin(b)*tz,ty,-math.sin(b)*tx+math.cos(b)*tz
        tx,ty,tz=(v+pb-pa for v,pb,pa in zip((tx,ty,tz),pivot_b,pivot_a))
        tx,ty,tz=tx,math.cos(a)*ty-math.sin(a)*tz,math.sin(a)*ty+math.cos(a)*tz
        return tuple(v+t+p for v,t,p in zip((x,y,z),(tx,ty,tz),pivot_a))
    x,y,z=x+tcp[0],y+tcp[1],z+tcp[2]
    x, y, z = x-pivot[0], y-pivot[1], z-pivot[2]
    a, b = math.radians(position[a_index]), math.radians(position[b_index])
    y, z = math.cos(a)*y+math.sin(a)*z, -math.sin(a)*y+math.cos(a)*z
    x, y = math.cos(b)*x+math.sin(b)*y, -math.sin(b)*x+math.cos(b)*y
    return x+pivot[0], y+pivot[1], z+pivot[2]

def deposition_length(start, end, a_index, b_index, pivot, layout, tcp=(0.,0.,0.), pivot_a=(0.,0.,0.), pivot_b=(0.,0.,0.)):
    count = max(1, int(math.ceil(max(abs(end[a_index]-start[a_index]), abs(end[b_index]-start[b_index]))/.1)))
    if count > 36000:
        raise ValueError('Rotary subdivision budget exceeded')
    previous = part_point(start, a_index, b_index, pivot, layout,tcp,pivot_a,pivot_b)
    distance = 0.
    for i in range(1, count+1):
        u = i/count
        q = [a+(b-a)*u for a, b in zip(start, end)]
        point = part_point(q, a_index, b_index, pivot, layout,tcp,pivot_a,pivot_b)
        distance += math.dist(previous, point)
        previous = point
    return distance

class FiveAxisFlow:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = False
        self.layout = 'BED'
        self.axis2 = 'C'
        self.pivot = (0., 0., 0.)
        self.tcp=self.pivot_a=self.pivot_b=(0.,0.,0.)
        self.max_flow = 10.
        self.native = None
        self.printer.register_event_handler('klippy:connect', self.connect)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('FIVEAXIS_ENABLE', self.enable)
        gcode.register_command('FIVEAXIS_DISABLE', self.disable)

    def connect(self):
        extruder = self.printer.lookup_object('extruder')
        self.native = extruder.check_move
        self.extruder = extruder
        extruder.check_move = self.check_move

    def enable(self, gcmd):
        self.layout = gcmd.get('LAYOUT', 'BED').upper()
        self.axis2 = gcmd.get('AXIS2', 'C').upper()
        if self.layout not in ('BED', 'HEAD') or self.axis2 not in ('B', 'C'):
            raise gcmd.error('Unsupported FiveAxisLab machine layout')
        self.pivot = tuple(gcmd.get_float('PIVOT_'+k, 0.) for k in 'XYZ')
        self.tcp=tuple(gcmd.get_float('TCP_'+k,0.) for k in 'XYZ')
        self.pivot_a=tuple(gcmd.get_float('PIVOT_A'+k,0.) for k in 'XYZ')
        self.pivot_b=tuple(gcmd.get_float('PIVOT_B'+k,0.) for k in 'XYZ')
        self.max_flow = gcmd.get_float('MAX_FLOW', 10., above=0.)
        self.indices()
        self.enabled = True

    def disable(self, gcmd):
        self.enabled = False

    def indices(self):
        axes = self.printer.lookup_object('toolhead').get_extra_axes()
        named = {axis.get_axis_gcode_id(): i for i, axis in enumerate(axes) if axis is not None}
        if 'A' not in named or self.axis2 not in named:
            raise self.printer.command_error('Register synchronized rotary axes before FIVEAXIS_ENABLE')
        return named['A'], named[self.axis2]

    def check_move(self, move, ea_index):
        if not self.enabled or move.axes_d[ea_index] <= 0:
            return self.native(move, ea_index)
        if not self.extruder.heater.can_extrude:
            raise self.printer.command_error('Extrude below minimum temp')
        a, b = self.indices()
        distance = deposition_length(move.start_pos,move.end_pos,a,b,self.pivot,self.layout,self.tcp,self.pivot_a,self.pivot_b)
        if distance < 1e-8:
            return self.native(move, ea_index)
        extrusion = move.axes_d[ea_index]
        area = extrusion*self.extruder.filament_area/distance
        maximum = self.extruder.max_extrude_ratio*self.extruder.filament_area
        if not math.isfinite(area) or area > maximum+1e-8:
            raise self.printer.command_error('Five-axis move exceeds maximum deposition cross-section')
        ratio = abs(move.axes_r[ea_index])
        if ratio:
            velocity = min(self.extruder.max_e_velocity, self.max_flow/self.extruder.filament_area)/ratio
            move.limit_speed(velocity, self.extruder.max_e_accel/ratio)

def load_config(config):
    return FiveAxisFlow(config)
