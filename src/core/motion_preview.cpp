#include "motion_preview.h"
#include "geometry.h"
#include "slicing.h"
namespace fiveaxis {
JointState rotary_preview_pose(const MachineProfile &m,const Vec3 &anchor,double a,double b) {
    JointState q; q.a=a;q.b=b; q.xyz=workpiece_pose(m,q)*anchor;return q;
}
std::array<Transform,2> rotary_axis_frames(const MachineProfile &m,const ToolheadAssembly &t,const JointState &q) {
    if(m.layout==Layout::RotaryBed)
        return {translation(m.bedPivot)*rotation_y(pi/2),translation(m.bedPivot)*rotation_x(q.a)};
    auto carriage=translation(controller_axes(m,t,q).xyz);
    // Query the same articulated-stage transform used for physical components.
    Component stage;stage.link=Link::RotaryStage;
    auto parent=component_pose(m,t,stage,q)*t.mount.inverse();
    return {carriage*translation(m.headPivotA)*rotation_y(pi/2),parent*translation(m.headPivotB)*rotation_x(-pi/2)};
}
Project axis_motion_demo(Layout layout){
    auto p=demo_project();p.machine.layout=layout;p.machine.minB=layout==Layout::RotaryBed?-pi:-75*pi/180;p.machine.maxB=-p.machine.minB;
    p.useDummyToolhead=true;p.dummyToolhead=make_dummy_toolhead(.6);p.process.nozzleDiameter=.6;
    p.path.clear();p.volume={};p.stress={};p.deposition={};p.invalidate();p.pathProcessFingerprint.clear();p.diagnostics.clear();
    Vec3 anchor(12,7,22);
    for(int i=0;i<=144;++i){
        double phase=i*2*pi/144;Sample s;s.joint=rotary_preview_pose(p.machine,anchor,35*pi/180*std::sin(phase),(layout==Layout::RotaryBed?85:40)*pi/180*std::sin(phase));
        s.position=part_position(p.machine,s.joint);s.direction=part_direction(p.machine,p.active_toolhead(),s.joint);s.role=Role::Travel;s.deltaE=0;s.speed=25;p.path.push_back(s);
    }
    refine_joint_motion(p);p.depositionValid=false;
    p.diagnostics<<"Non-printing tilt / roll demonstration. No material deposited; motion is unvalidated and export is blocked.";
    return p;
}
} // namespace fiveaxis
