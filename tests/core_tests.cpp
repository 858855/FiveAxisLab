#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/project.h"
#include "core/process.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include "core/motion_preview.h"
#include "core/profile.h"
#include <QFile>
#include <QTemporaryDir>
#include <QtTest/QtTest>
#include <cmath>
#include <map>
#include <algorithm>
#include <stdexcept>
using namespace fiveaxis;
class CoreTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
    void profile_normal_slicing_data(){QTest::addColumn<bool>("head");QTest::newRow("bed")<<false;QTest::newRow("head")<<true;}
    void profile_normal_slicing(){
        QFETCH(bool,head);auto p=spiral_profile_project(head?Layout::TiltingHead:Layout::RotaryBed);
        p.process.layerHeight=1.5;p.process.width=1.6;p.process.wallCount=1;p.process.infill=.2;try{slice_project(p);}catch(const std::exception &e){QFAIL(e.what());}
        QVERIFY(p.path.size()>100);QCOMPARE(profile_motion_violation(p),-1);double minimum=1,maximum=0;int extruding=0;
        for(size_t i=1;i<p.path.size();++i)if(p.path[i].deltaE>0){++extruding;auto section=profile_layer(p,p.path[i].layer);minimum=std::min(minimum,p.path[i].direction.dot(section.normal));maximum=std::max(maximum,std::abs(p.path[i].joint.a));
            auto q=simulation_at(p,(p.path[i-1].time+p.path[i].time)/2,false).joint;QVERIFY((part_direction(p.machine,p.active_toolhead(),q)-section.normal).norm()<1e-6);
        }
        QVERIFY(extruding>50);QVERIFY(minimum>1-1e-10);QVERIFY(maximum>10*pi/180);
        QTemporaryDir dir;save_project(p,dir.filePath("profile.5axis"));auto loaded=load_project(dir.filePath("profile.5axis"));QCOMPARE(loaded.fingerprint(),p.fingerprint());QCOMPARE(profile_motion_violation(loaded),-1);
        for(auto &s:p.path)if(s.deltaE>0){s.joint.a+=.1;break;}
        QVERIFY(profile_motion_violation(p)>=0);QCOMPARE(validate_motion(p).status,Status::Unresolved);
        p.report.status=Status::Clear;p.report.fingerprint=p.fingerprint();p.depositionValid=true;QVERIFY_EXCEPTION_THROWN(export_gcode(p),std::runtime_error);
    }
    void profile_guide_validation(){
        QVERIFY_EXCEPTION_THROWN(validate_profile_guide({}),std::runtime_error);
        std::vector<ProfileStation> guide{{Vec3::Zero(),Vec3::UnitZ()},{Vec3(0,0,4),Vec3::UnitZ()}};validate_profile_guide(guide);QCOMPARE(profile_length(guide),4.0);
        QVERIFY((profile_section(guide,2).center-Vec3(0,0,2)).norm()<1e-10);guide.back().normal=-Vec3::UnitZ();QVERIFY_EXCEPTION_THROWN(validate_profile_guide(guide),std::runtime_error);
        QTemporaryDir dir;QFile file(dir.filePath("guide.json"));QVERIFY(file.open(QIODevice::WriteOnly));file.write("{\"stations\":[{\"center\":[0,0,0],\"normal\":[0,0,1]},{\"center\":[0,0,4],\"normal\":[0,0,2]}]}");file.close();
        auto placed=import_profile_guide(file.fileName(),translation(Vec3(3,2,1))*rotation_y(.2));QVERIFY((placed.front().center-Vec3(3,2,1)).norm()<1e-10);QVERIFY((placed.front().normal-rotation_y(.2).linear()*Vec3::UnitZ()).norm()<1e-10);
        auto p=spiral_profile_project(Layout::RotaryBed);p.profileGuide.clear();QVERIFY_EXCEPTION_THROWN(slice_project(p),std::runtime_error);
    }
    void mounted_axis_kinematics(){
        for(auto layout:{Layout::RotaryBed,Layout::TiltingHead})for(auto axis:{Vec3::UnitZ().eval(),Vec3::UnitY().eval()}) {
            MachineProfile m;m.layout=layout;ToolheadAssembly t;t.mount=translation(Vec3(2,1,3))*rotation_y(.12)*rotation_z(.07);t.axis=axis;t.tcp=Vec3(.4,.2,-1);
            JointState known;known.a=.2;known.b=-.3;known.xyz=Vec3(12,7,30);Vec3 point=part_position(m,known),normal=part_direction(m,t,known);
            auto solutions=inverse_kinematics(m,point,normal,known,t);QVERIFY(!solutions.empty());QVERIFY((part_direction(m,t,solutions.front())-normal).norm()<1e-9);QVERIFY((part_position(m,solutions.front())-point).norm()<1e-9);
        }
    }
    void kinematics();
    void thin_obstacle();
    void rotary_sweep();
    void containment();
    void deposited_only();
    void tip_contact_and_heater();
    void fea_patch();
    void vtu_tensors();
    void vtu_components_and_mismatch();
    void step_roundtrip();
    void malformed_step();
    void project_roundtrip_and_stale_export();
    void simulation_backward();
    void controller_replay();
    void scalar_field_slice();
    void blocked_repair();
    void stale_kinematics();
    void clearance_threshold();
    void successful_travel_repair();
    void quadratic_tetra_mapping();
    void step_units();
    void curved_strength_layers();
    void playback_invariance();
    void unsupported_deposition();
    void nozzle_lift_and_penetration();
    void successful_orientation_repair();
    void cavity_meshing();
    void rotary_lever_bound();
    void unresolved_budget();
    void head_motor_interpolation();
    void conical_nozzle_contact();
    void bounded_interface();
    void fem_step_metadata();
    void orca_process_import();
    void process_roles_and_patterns();
    void reduced_and_disabled_supports();
    void dummy_toolhead_roundtrip_and_collision();
    void toolpaths_follow_joint_motion();
    void workspace_settings_geometry();
    void workspace_preset_import();
    void step_tessellation_options();
    void rotary_preview_calibration();
    void axis_motion_demonstrations();
};
void CoreTests::rotary_preview_calibration(){
    MachineProfile m;m.bedPivot=Vec3(3,-4,7);m.headPivotA=Vec3(2,-3,28);m.headPivotB=Vec3(4,2,16);
    ToolheadAssembly t;t.tcp=Vec3(1,2,-1);t.mount=translation(Vec3(3,-2,5))*rotation_z(.23);
    Vec3 anchor(18,11,32);
    for(auto layout:{Layout::RotaryBed,Layout::TiltingHead}){
        m.layout=layout;auto q=rotary_preview_pose(m,anchor,.4,-.6);auto axes=rotary_axis_frames(m,t,q);
        QVERIFY((part_position(m,q)-anchor).norm()<1e-9);
        Component nozzle;nozzle.link=Link::Nozzle;QVERIFY((component_pose(m,t,nozzle,q)*t.tcp-q.xyz).norm()<1e-9);
        for(const auto &frame:axes){QVERIFY(std::abs(frame.linear().determinant()-1)<1e-9);QVERIFY((frame.linear().transpose()*frame.linear()-Mat3::Identity()).norm()<1e-9);}
        QVERIFY((axes[0].linear().col(2)-Vec3::UnitX()).norm()<1e-9);
        if(layout==Layout::RotaryBed){
            QVERIFY((axes[0].translation()-m.bedPivot).norm()<1e-9);QVERIFY((axes[1].translation()-m.bedPivot).norm()<1e-9);
            QVERIFY((axes[1].linear().col(2)-rotation_x(q.a).linear()*Vec3::UnitZ()).norm()<1e-9);
        }else{
            auto carriage=controller_axes(m,t,q).xyz;auto a=translation(m.headPivotA)*rotation_x(q.a)*translation(-m.headPivotA);
            QVERIFY((axes[0].translation()-carriage-m.headPivotA).norm()<1e-9);
            QVERIFY((axes[1].translation()-carriage-a*m.headPivotB).norm()<1e-9);
            QVERIFY((axes[1].linear().col(2)-a.linear()*Vec3::UnitY()).norm()<1e-9);
        }
    }
}
void CoreTests::axis_motion_demonstrations(){
    QTemporaryDir dir;
    for(auto layout:{Layout::RotaryBed,Layout::TiltingHead}){
        auto p=axis_motion_demo(layout);QVERIFY(p.path.size()>100);bool tilt=false,rotation=false;
        for(const auto &s:p.path){QCOMPARE(s.deltaE,0.0);QCOMPARE(s.role,Role::Travel);QVERIFY(in_limits(p.machine,p.active_toolhead(),s.joint));tilt|=std::abs(s.joint.a)>.4;rotation|=std::abs(s.joint.b)>.5;}
        QVERIFY(tilt && rotation);QVERIFY(!p.report.exportable());QVERIFY(!p.depositionValid);QVERIFY_EXCEPTION_THROWN(export_gcode(p),std::runtime_error);
        auto file=dir.filePath(layout==Layout::RotaryBed?"bed.5axis":"head.5axis");save_project(p,file);auto loaded=load_project(file);QCOMPARE(loaded.fingerprint(),p.fingerprint());
        auto mid=simulation_at(loaded,loaded.path[loaded.path.size()/4].time,false);auto before=simulation_at(loaded,0,false);auto again=simulation_at(loaded,loaded.path[loaded.path.size()/4].time,false);
        QVERIFY(std::abs(mid.joint.a-before.joint.a)>.3);QVERIFY(std::abs(mid.joint.b-before.joint.b)>.3);QVERIFY((mid.joint.xyz-again.joint.xyz).norm()<1e-9);
    }
}
void CoreTests::workspace_preset_import() {
    QStringList notes;
    auto s=import_orca_process({{"top_surface_line_width","0.5"},{"support_line_width","0.6"},
        {"top_surface_density","85%"},{"top_surface_pattern","concentric"},{"top_shell_thickness","0.8"},
        {"alternate_extra_wall","1"},{"is_infill_first","1"},{"enable_overhang_speed","1"},{"initial_layer_infill_speed","55"},
        {"top_surface_speed","150"},{"small_perimeter_speed","50%"},{"small_perimeter_threshold","10"},
        {"xy_contour_compensation","0.1"},{"elefant_foot_compensation","0.15"},{"overhang_2_4_speed","50"},
        {"support_threshold_angle","30"},{"skirt_loops","2"},{"brim_type","outer_only"},{"brim_width","3"}},ProcessSettings{},notes);
    QCOMPARE(s.topWidth,.5); QCOMPARE(s.supportWidth,.6); QCOMPARE(s.topDensity,.85); QCOMPARE(s.topPattern,SurfacePattern::Concentric);
    QCOMPARE(s.topThickness,.8); QVERIFY(s.alternateExtraWall && s.infillFirst && s.slowOverhangs); QCOMPARE(s.firstLayerInfillSpeed,55.0);
    QCOMPARE(s.topSpeed,150.0); QCOMPARE(s.smallWallSpeed,.5); QCOMPARE(s.overhangSpeeds[1],50.0);
    QVERIFY(std::abs(s.supportAngle-pi/6)<1e-9); QCOMPARE(s.skirtLoops,2); QCOMPARE(s.brimType,2);
    QCOMPARE(process_json(read_process(process_json(s))),process_json(s));
    auto absolute=import_orca_process({{"small_perimeter_speed","15"}},s,notes); QCOMPARE(absolute.smallWallAbsoluteSpeed,15.0);
    auto j=process_json(s); j["layerHeight"]="bad"; QVERIFY_EXCEPTION_THROWN(read_process(j),std::runtime_error);
    j=process_json(s); j["overhangSpeeds"]=QJsonArray{1,-1,2,3,4}; QVERIFY_EXCEPTION_THROWN(read_process(j),std::runtime_error);
}
void CoreTests::workspace_settings_geometry() {
    try {
    auto base=demo_project(); base.part=box_mesh(Vec3(0,0,1),Vec3(8,8,2)); base.volume={};
    base.process.layerHeight=.4; base.process.firstLayerHeight=.2; base.process.width=.6; base.process.wallCount=1;
    base.process.supportMode=SupportMode::None; base.process.curved=false; base.process.infill=.3;
    slice_project(base);
    auto extreme=[](const Project &p,int layer) { double v=-1e9; for(const auto &s:p.path) if(s.role==Role::OuterWall && s.deltaE>0 && s.layer==layer) v=std::max(v,s.position.x()); return v; };
    auto compensated=base; compensated.process.contourCompensation=.15; compensated.process.elephantFoot=.05; slice_project(compensated);
    QVERIFY(std::abs(extreme(compensated,1)-extreme(base,1)-.15)<.025);
    QVERIFY(std::abs(extreme(compensated,0)-extreme(base,0)-.10)<.025);
    auto precise=base; precise.process.preciseZ=true; slice_project(precise); double top=0,height=0;
    for(const auto &s:precise.path) if(s.deltaE>0 && s.position.z()>top) {top=s.position.z();height=s.height;}
    QVERIFY(std::abs(top-2)<1e-5); QVERIFY(std::abs(height-.2)<1e-5);
    auto shell=base; shell.process.topThickness=.8; shell.process.topPattern=SurfacePattern::Concentric; shell.process.topWidth=.65; shell.process.topSpeed=17;
    shell.process.infillFirst=true; shell.process.alternateExtraWall=true; slice_project(shell);
    bool topFound=false,solidFound=false,extra=false; Role first=Role::Travel;
    for(const auto &s:shell.path) if(s.deltaE>0) {
        if(first==Role::Travel) first=s.role;
        if(s.role==Role::TopSurface) {topFound=true; QCOMPARE(s.width,.65); QCOMPARE(s.speed,17.0);}
        solidFound|=s.role==Role::SolidInfill; extra|=s.role==Role::Wall && s.layer%2==1;
    }
    QCOMPARE(first,Role::Infill); QVERIFY(topFound && solidFound && extra);
    auto noTop=shell; noTop.process.topDensity=0; slice_project(noTop);
    for(const auto &s:noTop.path) QVERIFY(s.role!=Role::TopSurface);
    auto adhesion=base; adhesion.process.skirtLoops=2; adhesion.process.skirtDistance=2; adhesion.process.brimType=2; adhesion.process.brimWidth=1.2; adhesion.process.brimGap=.1;
    slice_project(adhesion); double skirt=0,brim=0;
    for(const auto &s:adhesion.path) {if(s.role==Role::Skirt) skirt+=s.deltaE; if(s.role==Role::Brim) brim+=s.deltaE;}
    QVERIFY(skirt>0 && brim>0); QVERIFY(adhesion.deposition.checked);
    auto slow=base; slow.process.smallWallThreshold=50; slow.process.smallWallSpeed=.5; slow.process.outerWallSpeed=20; slice_project(slow);
    for(const auto &s:slow.path) if(s.role==Role::OuterWall && s.deltaE>0) QCOMPARE(s.speed,10.0);
    QTemporaryDir dir; adhesion.filamentName="ASA"; adhesion.filamentDensity=1.07; adhesion.filamentCost=29;
    auto fp=adhesion.fingerprint(); adhesion.filamentCost=39; QCOMPARE(adhesion.fingerprint(),fp);
    save_project(adhesion,dir.filePath("settings.5axis")); auto restored=load_project(dir.filePath("settings.5axis"));
    QCOMPARE(process_json(restored.process),process_json(adhesion.process)); QCOMPARE(restored.filamentName,QString("ASA")); QCOMPARE(restored.filamentCost,39.0);
    } catch(const std::exception &e) { QFAIL(e.what()); }
}
void CoreTests::step_tessellation_options() {
    QTemporaryDir dir; auto file=dir.filePath("fine.step"); write_demo_step(file,true,false,true);
    StepImportOptions fine; fine.angularDeflection=.03; fine.splitCompounds=true;
    auto a=load_step_assembly(file,fine); fine.angularDeflection=.6; fine.linearDeflection=.2; auto b=load_step_assembly(file,fine);
    size_t fa=0,fb=0; for(const auto &c:a.components) {fa+=c.mesh.triangles.size();QCOMPARE(c.mesh.error,.02);}
    for(const auto &c:b.components) {fb+=c.mesh.triangles.size();QCOMPARE(c.mesh.error,.2);}
    QVERIFY(fa>fb); QCOMPARE(a.sourceHash,b.sourceHash); QCOMPARE(a.components.size(),b.components.size());
    auto cached=load_step_assembly(file,fine); QCOMPARE(cached.components.front().mesh.triangles.size(),b.components.front().mesh.triangles.size());
    fine.splitCompounds=false; auto whole=load_step_assembly(file,fine); QVERIFY(whole.components.size()<b.components.size());
    fine.angularDeflection=0; QVERIFY_EXCEPTION_THROWN(load_step_assembly(file,fine),std::runtime_error);
}
void CoreTests::orca_process_import() {
    QStringList notes;
    auto s=import_orca_process({{"layer_height","0.2"},{"initial_layer_print_height","0.25"},
        {"line_width","0.45"},{"outer_wall_line_width","0.5"},{"wall_loops","3"},
        {"top_shell_layers","2"},{"bottom_shell_layers","1"},{"sparse_infill_density","35%"},
        {"sparse_infill_pattern","grid"},{"infill_direction","45"},{"seam_position","back"},
        {"outer_wall_speed","18"},{"travel_speed","130"},{"enable_support","0"},
        {"wall_sequence","inner wall/outer wall"},{"tree_support_branch_angle","45"}},ProcessSettings{},notes);
    QCOMPARE(s.layerHeight,.2); QCOMPARE(s.firstLayerHeight,.25); QCOMPARE(s.wallCount,3);
    QCOMPARE(s.infill,.35); QCOMPARE(s.infillPattern,InfillPattern::Grid); QCOMPARE(s.seam,SeamPosition::Back);
    QCOMPARE(s.topShellLayers,2); QCOMPARE(s.outerWallSpeed,18.0); QCOMPARE(s.travelSpeed,130.0);
    QVERIFY(!s.supports && s.supportMode==SupportMode::None && !s.outerWallFirst);
    QVERIFY(std::abs(s.infillAngle-pi/4)<1e-10); QVERIFY(notes.contains("Not imported: tree_support_branch_angle"));
    QCOMPARE(process_json(read_process(process_json(s))),process_json(s));
    QVERIFY_EXCEPTION_THROWN(import_orca_process({{"sparse_infill_pattern","gyroid"}},s,notes),std::runtime_error);
    QVERIFY_EXCEPTION_THROWN(import_orca_process({{"sparse_infill_density","bad"}},s,notes),std::runtime_error);
    QVERIFY_EXCEPTION_THROWN(import_orca_process({{"outer_wall_line_width","0.1"}},s,notes),std::runtime_error);
}
void CoreTests::process_roles_and_patterns() {
    auto p=demo_project(); p.part=box_mesh(Vec3(0,0,1),Vec3(8,8,2));
    p.process.layerHeight=.4; p.process.firstLayerHeight=.2; p.process.width=.6;
    p.process.firstLayerWidth=.7; p.process.outerWallWidth=.65; p.process.innerWallWidth=.6;
    p.process.infillWidth=.5; p.process.solidWidth=.55;
    p.process.firstLayerSpeed=12; p.process.outerWallSpeed=19; p.process.innerWallSpeed=27;
    p.process.infillSpeed=33; p.process.solidSpeed=21; p.process.travelSpeed=85;
    p.process.bottomShellLayers=1; p.process.topShellLayers=1; p.process.infillPattern=InfillPattern::Grid;
    p.process.outerWallFirst=false; p.process.supportMode=SupportMode::None;
    slice_project(p); bool outer=false,inner=false,infill=false,solid=false,first=false;
    Role firstExtrusion=Role::Travel;
    for(const auto &s:p.path) {
        if(s.deltaE<=0) { QCOMPARE(s.speed,85.0); continue; }
        if(firstExtrusion==Role::Travel) firstExtrusion=s.role;
        if(s.layer==0) { QCOMPARE(s.width,.7); QCOMPARE(s.height,.2); QCOMPARE(s.speed,12.0); first=true; continue; }
        if(s.role==Role::OuterWall) { QCOMPARE(s.width,.65); QCOMPARE(s.speed,19.0); outer=true; }
        if(s.role==Role::Wall) { QCOMPARE(s.width,.6); QCOMPARE(s.speed,27.0); inner=true; }
        if(s.role==Role::Infill) { QCOMPARE(s.width,.5); QCOMPARE(s.speed,33.0); infill=true; }
        if(s.role==Role::SolidInfill || s.role==Role::TopSurface) { QCOMPARE(s.width,s.role==Role::TopSurface?.6:.55); QCOMPARE(s.speed,s.role==Role::TopSurface?25.0:21.0); solid=true; }
    }
    QCOMPARE(firstExtrusion,Role::Wall); QVERIFY(first && outer && inner && infill && solid);
    QVERIFY(p.deposition.checked);
    auto directions=[&](InfillPattern pattern) {
        auto q=p; q.process.infillPattern=pattern; q.process.infillAngle=pi/4; q.process.topShellLayers=q.process.bottomShellLayers=0;
        slice_project(q); bool positive=false,negative=false;
        for(size_t i=1;i<q.path.size();++i) if(q.path[i].role==Role::Infill && q.path[i].deltaE>0 && q.path[i].layer==0) {
            Vec3 d=q.path[i].position-q.path[i-1].position; positive|=d.x()*d.y()>1e-6; negative|=d.x()*d.y()<-1e-6;
        }
        return std::make_pair(positive,negative);
    };
    auto grid=directions(InfillPattern::Grid), lines=directions(InfillPattern::Rectilinear);
    QVERIFY(grid.first && grid.second); QVERIFY(lines.first && !lines.second);
    p.process.infill=.5;
    auto stale=validate_motion(p); QCOMPARE(stale.status,Status::Unresolved);
    QVERIFY(stale.events.front().reason.contains("slice again"));
}
static Project overhang_fixture() {
    auto p=demo_project(); p.path.clear(); p.volume={};
    std::map<std::array<int,3>,int> ids;
    auto id=[&](std::array<int,3> v) { auto [it,created]=ids.emplace(v,int(p.volume.vertices.size()));
        if(created) p.volume.vertices.push_back(Vec3(v[0]-3,v[1]-2,v[2])); return it->second; };
    for(int z=0;z<3;++z) for(int y=0;y<4;++y) for(int x=0;x<6;++x) {
        if(z<2 && (x<2 || x>=4)) continue;
        std::array<int,3> axes={0,1,2};
        do { std::array<int,3> v={x,y,z}; std::vector<int> n={id(v)};
            for(int axis:axes) { ++v[axis]; n.push_back(id(v)); }
            p.volume.cells.push_back({n});
        } while(std::next_permutation(axes.begin(),axes.end()));
    }
    p.part=volume_surface(p.volume); p.process.layerHeight=.5; p.process.width=.6;
    p.process.wallCount=1; p.process.infill=.4; p.process.maximumBridge=.4;
    return p;
}
void CoreTests::reduced_and_disabled_supports() {
    auto normal=overhang_fixture(),reduced=normal,none=normal;
    reduced.process.supportMode=SupportMode::Reduced; reduced.process.supportReduction=2;
    none.process.supportMode=SupportMode::None;
    slice_project(normal); slice_project(reduced); slice_project(none);
    auto amount=[](const Project &p) { double volume=0; for(const auto &s:p.path) if(s.role==Role::Support) volume+=std::max(0.0,s.deltaE); return volume; };
    QVERIFY(amount(normal)>0); QVERIFY(amount(reduced)>0); QVERIFY(amount(reduced)<amount(normal));
    QCOMPARE(amount(none),0.0); QVERIFY(!none.deposition.unsupportedMoves.empty()); QVERIFY(!none.depositionValid);
    none.report=validate_motion(none); QVERIFY(!none.report.exportable());
    QVERIFY_EXCEPTION_THROWN(export_gcode(none),std::runtime_error);
}
void CoreTests::dummy_toolhead_roundtrip_and_collision() {
    QTemporaryDir dir; write_demo_step(dir.filePath("head.step"));
    auto p=demo_project(); p.toolhead=load_step_assembly(dir.filePath("head.step"));
    auto cadHash=p.toolhead.sourceHash; auto cad=p.toolhead.sourceStep;
    p.dummyToolhead=make_dummy_toolhead(.8); p.dummyToolhead.contactHeight=.5;
    p.process.supportMode=SupportMode::Reduced; p.process.outerWallSpeed=16;
    p.useDummyToolhead=true;
    auto file=dir.filePath("dummy.5axis"); save_project(p,file); auto q=load_project(file);
    QVERIFY(q.useDummyToolhead); QCOMPARE(q.toolhead.sourceHash,cadHash); QCOMPARE(q.toolhead.sourceStep,cad);
    QCOMPARE(q.fingerprint(),p.fingerprint()); QCOMPARE(q.active_toolhead().components.size(),size_t(4));
    QCOMPARE(q.process.outerWallSpeed,16.0); QCOMPARE(q.process.supportMode,SupportMode::Reduced);
    q.fixtures.push_back({"Dummy heater obstacle",box_mesh(Vec3(0,0,22),Vec3(2,2,2)),false});
    CollisionScene scene(q); JointState pose; pose.xyz=Vec3(0,0,10);
    auto report=scene.check_pose(pose,false,pose.xyz); QCOMPARE(report.status,Status::Collision);
    auto fingerprint=q.fingerprint(); q.useDummyToolhead=false; QVERIFY(q.fingerprint()!=fingerprint);
    QCOMPARE(q.active_toolhead().sourceHash,cadHash);
    auto legacy=project_json(p); legacy.remove("dummyToolhead"); legacy.remove("useDummyToolhead");
    legacy["process"]=QJsonObject{{"layerHeight",.5},{"width",.8},{"supports",false}};
    auto old=project_from_json(legacy); QVERIFY(!old.useDummyToolhead); QVERIFY(!old.process.supports);
}
void CoreTests::toolpaths_follow_joint_motion() {
    Project p; Sample a,b; a.joint.xyz=Vec3(10,0,5); a.position=part_position(p.machine,a.joint);
    b=a; b.joint.b=pi/2; b.position=part_position(p.machine,b.joint); b.time=1; b.layer=4;
    p.path={a,b}; auto segments=simulation_toolpaths(p); QVERIFY(segments.size()>10);
    QVERIFY((segments.front().start-a.position).norm()<1e-10); QVERIFY((segments.back().end-b.position).norm()<1e-10);
    bool curved=false;
    for(auto &s:segments) { QCOMPARE(s.role,Role::Travel); QCOMPARE(s.layer,4); QCOMPARE(s.move,1);
        Vec3 actual=part_position(p.machine,interpolate_motion(p.machine,p.active_toolhead(),a.joint,b.joint,(s.begin+s.finish)/2));
        QVERIFY((actual-(s.start+s.end)/2).norm()<=p.machine.chordTolerance);
        curved|=(s.start-(a.position+(b.position-a.position)*s.begin)).norm()>.1;
    }
    QVERIFY(curved); auto late=simulation_at(p,.8),early=simulation_at(p,.2),again=simulation_at(p,.8);
    QVERIFY((late.joint.xyz-again.joint.xyz).norm()<1e-10); QVERIFY(early.fraction<late.fraction);
    auto repeated=simulation_toolpaths(p); QCOMPARE(repeated.size(),segments.size());
}
static Project small() {
    Project p;
    p.machine.clearance = .05;
    Component c;
    c.id = "probe";
    c.name = "Probe";
    c.mesh = box_mesh(Vec3::Zero(), Vec3(.2, .2, .2));
    p.toolhead.components.push_back(c);
    p.depositionValid = true;
    return p;
}
void CoreTests::kinematics() {
    for (Layout layout : {Layout::RotaryBed, Layout::TiltingHead}) {
        MachineProfile m;
        m.layout = layout;
        ToolheadAssembly t;
        for (int i = 0; i < 100; ++i) {
            JointState q;
            q.a = .5 * std::sin(i);
            q.b = .5 * std::cos(i);
            q.xyz = Vec3(10 + std::sin(i) * 5, 20, 30);
            Vec3 p = part_position(m, q), n = part_direction(m, t, q);
            auto solutions = inverse_kinematics(m, p, n, q);
            QVERIFY(!solutions.empty());
            QVERIFY((part_position(m, solutions[0]) - p).norm() < 1e-8);
            QVERIFY((part_direction(m, t, solutions[0]) - n).norm() < 1e-8);
            Component c;
            c.link = Link::Nozzle;
            Transform tr = component_pose(m, t, c, q);
            QVERIFY((tr * t.tcp - q.xyz).norm() < 1e-8);
        }
    }
}
void CoreTests::thin_obstacle() {
    auto p = small();
    p.fixtures.push_back({"Thin plate", box_mesh(Vec3(0, 0, 5), Vec3(.02, 1, 1)), false});
    CollisionScene scene(p);
    JointState a, b;
    a.xyz = Vec3(-5, 0, 5);
    b.xyz = Vec3(5, 0, 5);
    QCOMPARE(scene.check_pose(a, false, a.xyz).status, Status::Clear);
    QCOMPARE(scene.check_pose(b, false, b.xyz).status, Status::Clear);
    CollisionReport report;
    report.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, report));
    QVERIFY(!report.events.empty());
}
void CoreTests::rotary_sweep() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    p.machine.headPivotA = p.machine.headPivotB = Vec3::Zero();
    p.toolhead.components[0].mesh = box_mesh(Vec3(10, 0, 0), Vec3(.2, .2, .2));
    p.fixtures.push_back({"Rotary obstacle", box_mesh(Vec3(10, 0, 0), Vec3(.1, .5, .5)), false});
    CollisionScene scene(p);
    JointState a, b;
    a.xyz = b.xyz = Vec3::Zero();
    a.b = -.8;
    b.b = .8;
    QCOMPARE(scene.check_pose(a, false, Vec3::Zero()).status, Status::Clear);
    QCOMPARE(scene.check_pose(b, false, Vec3::Zero()).status, Status::Clear);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, Vec3::Zero(), Vec3::Zero(), 1, 0, 1, r));
}
void CoreTests::containment() {
    auto p = small();
    p.toolhead.components[0].mesh = box_mesh(Vec3::Zero(), Vec3(10, 10, 10));
    p.fixtures.push_back({"Contained object", box_mesh(Vec3::Zero(), Vec3(1, 1, 1)), false});
    CollisionScene scene(p);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Collision);
}
void CoreTests::deposited_only() {
    auto p = small();
    p.part = box_mesh(Vec3::Zero(), Vec3(20, 20, 20));
    CollisionScene scene(p);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Clear);
    scene.deposit(Vec3(-1, 0, 0), Vec3(1, 0, 0), .4, .2, Role::Wall);
    QCOMPARE(scene.check_pose(q, false, Vec3::Zero()).status, Status::Collision);
}
void CoreTests::tip_contact_and_heater() {
    auto p = small();
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .15, 2);
    p.toolhead.components[0].nozzle = true;
    p.toolhead.contactHeight = .6;
    CollisionScene scene(p);
    scene.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    JointState q;
    q.xyz = Vec3(0, 0, .1);
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Clear);
    Component heater;
    heater.name = "Heater";
    heater.mesh = box_mesh(Vec3::Zero(), Vec3(.5, .5, .5));
    p.toolhead.components.push_back(heater);
    CollisionScene next(p);
    next.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    QCOMPARE(next.check_pose(q, true, q.xyz).status, Status::Collision);
    p.toolhead.components.pop_back();
    CollisionScene neighboring(p);
    neighboring.deposit(Vec3(-.4, 0, 1), Vec3(.4, 0, 1), .4, .2, Role::Wall);
    QCOMPARE(neighboring.check_pose(q, true, q.xyz).status, Status::Collision);
}
static VolumeMesh unit_tetra() {
    VolumeMesh mesh;
    mesh.vertices = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
    mesh.cells.push_back({{0, 1, 2, 3}});
    return mesh;
}
void CoreTests::fea_patch() {
    auto mesh = unit_tetra();
    LoadCase load;
    load.fixedNodes = {0, 1, 2};
    load.nodalForces = {{3, Vec3(0, 0, -1)}};
    load.material.young = 1000;
    load.material.poisson = .25;
    auto result = solve_fea(mesh, load);
    QVERIFY(std::abs(result.displacement[3].z() + .005) < 1e-10);
    QVERIFY(std::abs(result.stress.tensors[0](2, 2) + 6) < 1e-8);
    QVERIFY(result.compliance > 0);
    load.fixedNodes = {0};
    bool failed = false;
    try {
        solve_fea(mesh, load);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
}
static void write_vtu(const QString &filename, const QString &arrays, bool mismatched = false) {
    QFile f(filename);
    QVERIFY(f.open(QIODevice::WriteOnly));
    QString xml =
        QString("<VTKFile type='UnstructuredGrid' version='0.1' "
                "byte_order='LittleEndian'><UnstructuredGrid><Piece NumberOfPoints='4' "
                "NumberOfCells='1'><Points><DataArray type='Float64' NumberOfComponents='3' format='ascii'>0 "
                "0 0 %1 0 0 0 1 0 0 0 1</DataArray></Points><Cells><DataArray type='Int32' "
                "Name='connectivity' format='ascii'>0 1 2 3</DataArray><DataArray type='Int32' "
                "Name='offsets' format='ascii'>4</DataArray><DataArray type='UInt8' Name='types' "
                "format='ascii'>10</DataArray></Cells><PointData>%2</PointData></Piece></UnstructuredGrid></"
                "VTKFile>")
            .arg(mismatched ? 2 : 1)
            .arg(arrays);
    f.write(xml.toUtf8());
}
void CoreTests::vtu_tensors() {
    QTemporaryDir dir;
    QString file = dir.filePath("tensor.vtu");
    write_vtu(file, "<DataArray type='Float64' Name='stress' NumberOfComponents='6' format='ascii'>10 1 1 0 "
                    "0 0 10 1 1 0 0 0 10 1 1 0 0 0 10 1 1 0 0 0</DataArray>");
    VtuOptions o;
    o.array = "stress";
    o.stressScale = 2;
    o.alignment = rotation_z(pi / 2);
    auto field = import_vtu({file}, o);
    QVERIFY(field.directional);
    bool covered;
    auto t = field.sample(Vec3(-.1, .1, .1), &covered);
    QVERIFY(covered);
    QVERIFY(std::abs(t(1, 1) - 20) < 1e-8);
    QVERIFY(std::abs(principal_tensile_direction(t).y()) > .999);
    field.sample(Vec3(50, 0, 0), &covered);
    QVERIFY(!covered);
}
void CoreTests::vtu_components_and_mismatch() {
    QTemporaryDir dir;
    QStringList files;
    for (int i = 0; i < 6; ++i) {
        auto file = dir.filePath(QString::number(i) + ".vtu");
        write_vtu(file,
                  QString("<DataArray type='Float64' Name='value' format='ascii'>%1 %1 %1 %1</DataArray>")
                      .arg(i + 1));
        files << file;
    }
    VtuOptions o;
    auto field = import_vtu(files, o);
    QVERIFY(field.directional);
    QCOMPARE(field.tensors[0](0, 1), 4.0);
    write_vtu(files[5], "<DataArray type='Float64' Name='value' format='ascii'>1 1 1 1</DataArray>", true);
    bool failed = false;
    try {
        import_vtu(files, o);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
    auto scalar = import_vtu({files[0]}, o);
    QVERIFY(!scalar.directional);
    QCOMPARE(scalar.scalar.size(), size_t(4));
}
void CoreTests::step_roundtrip() {
    QTemporaryDir dir;
    auto path = dir.filePath("nested.step");
    write_demo_step(path, true);
    auto model = load_step_assembly(path);
    QCOMPARE(model.components.size(), size_t(5));
    std::set<QString> ids;
    double maxX = -1e100;
    for (auto &c : model.components) {
        ids.insert(c.id);
        maxX = std::max(maxX, c.mesh.bounds().max().x());
        QVERIFY(!c.mesh.empty());
    }
    QCOMPARE(ids.size(), size_t(5));
    QVERIFY(maxX > 30);
    QVERIFY(!model.sourceStep.isEmpty());
    QCOMPARE(model.sourceHash.size(), 64);
}
void CoreTests::malformed_step() {
    QTemporaryDir dir;
    auto path = dir.filePath("bad.step");
    QFile file(path);
    QVERIFY(file.open(QIODevice::WriteOnly));
    file.write("not a STEP file");
    file.close();
    bool failed = false;
    try {
        load_step_assembly(path);
    } catch (...) {
        failed = true;
    }
    QVERIFY(failed);
}
void CoreTests::project_roundtrip_and_stale_export() {
    QTemporaryDir dir;
    auto p = small();
    Sample a, b;
    a.joint.xyz = a.position = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    p.path = {a, b};
    p.report = validate_motion(p);
    QVERIFY(p.report.exportable());
    auto file = dir.filePath("project.5axis");
    save_project(p, file);
    auto loaded = load_project(file);
    QCOMPARE(loaded.fingerprint(), p.fingerprint());
    QVERIFY(!export_gcode(loaded).isEmpty());
    loaded.machine.clearance += .1;
    bool blocked = false;
    try {
        export_gcode(loaded);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::simulation_backward() {
    auto p = small();
    Sample a, b, c;
    a.position = a.joint.xyz = Vec3(0, 0, 1);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 1);
    b.deltaE = .1;
    b.time = 1;
    c = b;
    c.position = c.joint.xyz = Vec3(2, 0, 1);
    c.time = 2;
    p.path = {a, b, c};
    auto later = simulation_at(p, 1.5), earlier = simulation_at(p, .5), again = simulation_at(p, 1.5);
    QCOMPARE(later.deposited.size(), size_t(2));
    QCOMPARE(earlier.deposited.size(), size_t(1));
    QVERIFY((earlier.deposited[0].second - Vec3(.5, 0, 1)).norm() < 1e-9);
    QCOMPARE(later.joint.xyz.x(), again.joint.xyz.x());
}
void CoreTests::controller_replay() {
    for (auto layout : {Layout::RotaryBed, Layout::TiltingHead})
        for (auto controller : {Controller::Duet, Controller::Klipper}) {
            auto p = small();
            p.machine.layout = layout;
            p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, .5), .1, 1);
            p.toolhead.components[0].nozzle = true;
            p.machine.controller = controller;
            Sample a, b;
            a.joint.xyz = a.position = Vec3(0, 0, 5);
            b = a;
            b.joint.a = .1;
            b.joint.b = .2;
            b.position = part_position(p.machine, b.joint);
            b.direction = part_direction(p.machine, p.toolhead, b.joint);
            b.time = 1;
            b.deltaE = .02;
            b.role = Role::Wall;
            p.path = {a, b};
            p.report = validate_motion(p);
            QVERIFY(p.report.exportable());
            auto g = export_gcode(p);
            auto replay = replay_gcode(p, g);
            QVERIFY((replay.path.back().joint.xyz - b.joint.xyz).norm() < 1e-6);
            QVERIFY(std::abs(replay.path.back().joint.a - b.joint.a) < 1e-8);
            QVERIFY(std::abs(replay.path.back().deltaE - b.deltaE) < 1e-8);
        }
}
void CoreTests::head_motor_interpolation() {
    MachineProfile m;
    m.layout = Layout::TiltingHead;
    ToolheadAssembly t;
    t.tcp = Vec3(3, 0, 2);
    JointState a, b;
    a.xyz = b.xyz = Vec3(10, 10, 20);
    b.a = .4;
    b.b = .5;
    auto left = controller_axes(m, t, a), right = controller_axes(m, t, b),
         mid = interpolate_motion(m, t, a, b, .5);
    QVERIFY((controller_axes(m, t, mid).xyz - (left.xyz + right.xyz) / 2).norm() < 1e-9);
    QVERIFY((mid.xyz - a.xyz).norm() > .01);
    Component c;
    QVERIFY((component_pose(m, t, c, mid) * t.tcp - mid.xyz).norm() < 1e-9);
    m.layout = Layout::RotaryBed;
    m.minXYZ = Vec3(-10, -10, -10);
    m.maxXYZ = Vec3(10, 10, 10);
    t.tcp = Vec3(30, 0, 0);
    a.xyz = Vec3(35, 0, 5);
    a.a = a.b = 0;
    QVERIFY(in_limits(m, t, a));
    QVERIFY(!inverse_kinematics(m, a.xyz, Vec3::UnitZ(), a, t).empty());
}
void CoreTests::conical_nozzle_contact() {
    auto p = small();
    auto &c = p.toolhead.components[0];
    c.mesh = cylinder_mesh(Vec3(0, 0, 1), .1, 2);
    c.nozzle = true;
    for (auto &point : c.mesh.vertices)
        if (point.z() > 1) {
            point.x() *= 8;
            point.y() *= 8;
        }
    p.toolhead.contactHeight = .75;
    p.toolhead.contactRadius = .55;
    CollisionScene scene(p);
    scene.deposit(Vec3(-.4, 0, 0), Vec3(.4, 0, 0), .4, .2, Role::Wall);
    JointState q;
    q.xyz = Vec3::Zero();
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Clear);
    scene.deposit(Vec3(-.4, 0, 1), Vec3(.4, 0, 1), .4, .2, Role::Wall);
    QCOMPARE(scene.check_pose(q, true, q.xyz).status, Status::Collision);
}
void CoreTests::bounded_interface() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    auto &a = p.toolhead.components[0];
    a.id = "carriage";
    a.link = Link::Carriage;
    a.mesh = box_mesh(Vec3::Zero(), Vec3(4, 4, 4));
    Component b;
    b.id = b.name = "stage";
    b.mesh = box_mesh(Vec3::Zero(), Vec3(1, 1, 1));
    b.link = Link::Nozzle;
    p.toolhead.components.push_back(b);
    JointState q;
    q.xyz = Vec3(0, 0, 5);
    CollisionScene original(p);
    QCOMPARE(original.check_pose(q, false, q.xyz).status, Status::Collision);
    p.toolhead.interfaces.push_back({"carriage", "stage", Vec3::Zero(), Vec3(2, 2, 2)});
    CollisionScene allowed(p);
    QCOMPARE(allowed.check_pose(q, false, q.xyz).status, Status::Clear);
    p.toolhead.components[1].mesh = box_mesh(Vec3(1.4, 0, 0), Vec3(1, 1, 1));
    CollisionScene outside(p);
    QCOMPARE(outside.check_pose(q, false, q.xyz).status, Status::Collision);
}
void CoreTests::fem_step_metadata() {
    QTemporaryDir dir;QStringList files;
    for(int i=0;i<6;++i){QString file=dir.filePath(QString::number(i)+".vtu");
        write_vtu(file,"<DataArray type='Float64' Name='value' format='ascii'>1 1 1 1</DataArray>");
        QFile input(file);QVERIFY(input.open(QIODevice::ReadOnly));auto xml=input.readAll();input.close();
        xml.replace("<UnstructuredGrid>",QByteArray("<UnstructuredGrid><FieldData><DataArray type='Float64' Name='TimeValue' NumberOfTuples='1' format='ascii'>")+
                    QByteArray::number(i==5?2:1)+"</DataArray></FieldData>");
        QVERIFY(input.open(QIODevice::WriteOnly|QIODevice::Truncate));input.write(xml);input.close();files<<file;
    }
    VtuOptions options;bool rejected=false;try{import_vtu(files,options);}catch(...){rejected=true;}QVERIFY(rejected);
    options.resultStep="2";rejected=false;try{import_vtu({files[0]},options);}catch(...){rejected=true;}QVERIFY(rejected);
}
void CoreTests::scalar_field_slice() {
    auto p = demo_project();
    p.part = box_mesh(Vec3(0, 0, 1), Vec3(6, 6, 2));
    p.process.infill = .3;
    slice_project(p);
    QVERIFY(p.path.size() > 10);
    QVERIFY(p.depositionValid);
    for (auto &s : p.path) {
        QVERIFY(s.position.allFinite());
        QVERIFY(in_limits(p.machine, s.joint));
    }
}
void CoreTests::blocked_repair() {
    auto p = small();
    p.fixtures.push_back({"Enclosure", box_mesh(Vec3(0, 0, 5), Vec3(20, 20, 20)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 5);
    b.time = 1;
    p.path = {a, b};
    QVERIFY(!repair_motion(p));
    bool blocked = false;
    try {
        export_gcode(p);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::stale_kinematics() {
    auto p = small();
    Sample a, b;
    a.position = a.joint.xyz = Vec3(5, 0, 5);
    b = a;
    b.joint.a = .4;
    b.position = part_position(p.machine, b.joint);
    b.direction = part_direction(p.machine, p.toolhead, b.joint);
    b.time = 1;
    p.path = {a, b};
    p.machine.bedPivot = Vec3(50, 20, 0);
    QCOMPARE(validate_motion(p).status, Status::Unresolved);
}
void CoreTests::clearance_threshold() {
    auto p = small();
    p.fixtures.push_back({"Plate", box_mesh(Vec3(0, 0, -.6), Vec3(2, 2, 1)), false});
    JointState q;
    q.xyz = Vec3(0, 0, .1);
    p.machine.clearance = .09;
    CollisionScene scene(p);
    QCOMPARE(scene.check_pose(q, false, q.xyz).status, Status::Clear);
    p.machine.clearance = .11;
    QCOMPARE(scene.check_pose(q, false, q.xyz).status, Status::Collision);
    p.toolhead.components[0].mesh.error = .03;
    p.machine.clearance = .09;
    CollisionScene uncertain(p);
    QCOMPARE(uncertain.check_pose(q, false, q.xyz).status, Status::Collision);
}
void CoreTests::successful_travel_repair() {
    auto p = small();
    p.machine.minXYZ = Vec3(-6, -6, 3);
    p.machine.maxXYZ = Vec3(6, 6, 7);
    p.machine.minA = p.machine.minB = -.2;
    p.machine.maxA = p.machine.maxB = .2;
    p.fixtures.push_back({"Thin obstacle", box_mesh(Vec3(0, 0, 5), Vec3(.1, 2, 2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-5, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    b.deltaE = -.8;
    b.role = Role::Retract;
    p.path = {a, b};
    QVERIFY(repair_motion(p));
    QVERIFY(p.path.size() > 2);
    double E = 0;
    for (auto &s : p.path)
        E += s.deltaE;
    QVERIFY(std::abs(E + .8) < 1e-9);
    auto again = small();
    again.machine = p.machine;
    again.fixtures = p.fixtures;
    again.path = {a, b};
    QVERIFY(repair_motion(again));
    QCOMPARE(again.path.size(), p.path.size());
    for (size_t i = 0; i < p.path.size(); ++i)
        QVERIFY((again.path[i].joint.xyz - p.path[i].joint.xyz).norm() < 1e-9);
}
void CoreTests::quadratic_tetra_mapping() {
    auto mesh = unit_tetra();
    const int edges[][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
    for (auto &e : edges)
        mesh.vertices.push_back((mesh.vertices[e[0]] + mesh.vertices[e[1]]) / 2);
    mesh.vertices[4].z() += .1;
    mesh.cells[0].nodes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Eigen::Vector4d w;
    w << .4, .2, .3, .1;
    Vec3 point = Vec3::Zero();
    for (int j = 0; j < 4; ++j)
        point += w[j] * (2 * w[j] - 1) * mesh.vertices[j];
    for (int j = 0; j < 6; ++j)
        point += 4 * w[edges[j][0]] * w[edges[j][1]] * mesh.vertices[4 + j];
    Eigen::Vector4d mapped;
    QVERIFY(barycentric(mesh, mesh.cells[0], point, mapped));
    QVERIFY((mapped - w).norm() < 1e-7);
}
void CoreTests::step_units() {
    QTemporaryDir dir;
    auto path = dir.filePath("inches.step");
    write_demo_step(path, false, true);
    auto model = load_step_assembly(path);
    QVERIFY(!model.sourceUnits.isEmpty());
    double max = 0;
    for (auto &c : model.components)
        max = std::max(max, c.mesh.bounds().max().z());
    QVERIFY(std::abs(max - 24) < 1e-4);
    auto cached = load_step_assembly(path);
    QCOMPARE(cached.components.size(), model.components.size());
    QVERIFY(!cached.components.front().renderMesh.empty());
}
void CoreTests::curved_strength_layers() {
    auto p = demo_project();
    p.part = box_mesh(Vec3(0, 0, 4), Vec3(5, 5, 8));
    p.process.layerHeight = .4;
    p.process.width = .6;
    p.volume = tetrahedralize(p.part, .5);
    p.stress.mesh = p.volume;
    p.stress.directional = true;
    Vec3 direction = Vec3(1, 0, 1).normalized();
    Mat3 tensor = Mat3::Identity() + 10 * direction * direction.transpose();
    p.stress.tensors.assign(p.volume.vertices.size(), tensor);
    slice_project(p);
    double tilt = 0;
    for (auto &s : p.path)
        tilt = std::max(tilt, std::abs(s.joint.a));
    QVERIFY(tilt > pi / 180);
    QVERIFY(p.deposition.checked);
    QVERIFY(p.deposition.coverage > .9);
}
void CoreTests::playback_invariance() {
    auto p = small();
    p.fixtures.push_back({"Plate", box_mesh(Vec3(0, 0, 5), Vec3(.01, 2, 2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-5, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(5, 0, 5);
    b.time = 1;
    p.path = {a, b};
    auto r = validate_motion(p);
    for (double speed : {.25, 1.0, 10.0})
        for (double t = 0; t < 1; t += speed / 60)
            simulation_at(p, t, false);
    auto after = validate_motion(p);
    QCOMPARE(r.events.size(), after.events.size());
    QCOMPARE(r.checkedIntervals, after.checkedIntervals);
    QCOMPARE(r.events[0].time, after.events[0].time);
}
void CoreTests::unsupported_deposition() {
    auto p = small();
    Sample a, b;
    a.position = a.joint.xyz = Vec3(0, 0, 5);
    b = a;
    b.position = b.joint.xyz = Vec3(1, 0, 5);
    b.time = 1;
    b.deltaE = bead_area(b.width, b.height) / (pi * std::pow(1.75 / 2, 2));
    b.role = Role::Wall;
    p.path = {a, b};
    check_deposition(p);
    QVERIFY(!p.depositionValid);
    QVERIFY(!p.deposition.unsupportedMoves.empty());
    p.report = validate_motion(p);
    bool blocked = false;
    try {
        export_gcode(p);
    } catch (...) {
        blocked = true;
    }
    QVERIFY(blocked);
}
void CoreTests::nozzle_lift_and_penetration() {
    auto p = small();
    p.machine.clearance = .25;
    p.toolhead.components[0].nozzle = true;
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .15, 2);
    p.toolhead.contactHeight = .75;
    CollisionScene scene(p);
    scene.deposit(Vec3(-1, 0, .5), Vec3(1, 0, .5), .4, .2, Role::Wall);
    JointState a, b;
    a.xyz = Vec3(0, 0, .5001);
    b.xyz = Vec3(0, 0, 3);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, r));
    a.xyz.z() = .4;
    QCOMPARE(scene.check_pose(a, false, a.xyz).status, Status::Collision);
}
void CoreTests::successful_orientation_repair() {
    auto p = small();
    p.machine.layout = Layout::TiltingHead;
    p.toolhead.components[0].mesh = cylinder_mesh(Vec3(0, 0, 1), .1, 2);
    p.toolhead.components[0].nozzle = true;
    Component shroud;
    shroud.name = "Shroud";
    shroud.mesh = box_mesh(Vec3(1.5, 0, 8), Vec3(.4, .4, .4));
    p.toolhead.components.push_back(shroud);
    p.fixtures.push_back({"Bed", box_mesh(Vec3(0, 0, -.5), Vec3(20, 20, 1)), true});
    p.fixtures.push_back({"Obstacle", box_mesh(Vec3(1.5, 0, 8.2), Vec3(.2, .2, .2)), false});
    Sample a, b;
    a.position = a.joint.xyz = Vec3(-2, 0, .2);
    b = a;
    b.position = b.joint.xyz = Vec3(0, 0, .2);
    b.time = 1;
    b.role = Role::Wall;
    b.deltaE = 2 * bead_area(b.width, b.height) / (pi * std::pow(1.75 / 2, 2));
    p.path = {a, b};
    QCOMPARE(validate_motion(p).status, Status::Collision);
    QVERIFY(repair_motion(p));
    QVERIFY(p.path.back().direction.dot(Vec3::UnitZ()) >= std::cos(p.process.directionCone) - 1e-7);
}
void CoreTests::cavity_meshing() {
    Mesh m = box_mesh(Vec3::Zero(), Vec3(6, 6, 6));
    auto inner = box_mesh(Vec3::Zero(), Vec3(2, 2, 2));
    int offset = int(m.vertices.size());
    m.vertices.insert(m.vertices.end(), inner.vertices.begin(), inner.vertices.end());
    for (auto face : inner.triangles) {
        std::swap(face[1], face[2]);
        m.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
    }
    auto volume = tetrahedralize(m, 2);
    QVERIFY(!volume.cells.empty());
    for (auto &cell : volume.cells) {
        Vec3 center = Vec3::Zero();
        for (int i = 0; i < 4; ++i)
            center += volume.vertices[cell.nodes[i]] / 4;
        QVERIFY(center.cwiseAbs().maxCoeff() > 1 - 1e-6);
    }
}
void CoreTests::rotary_lever_bound() {
    MachineProfile m;
    ToolheadAssembly t;
    JointState a, b;
    a.xyz = Vec3(1000, 0, 0);
    b = a;
    b.b = .1;
    double actual = (part_position(m, b) - part_position(m, a)).norm();
    QVERIFY(actual <= movement_bound(m, t, a, b, 1));
}
void CoreTests::unresolved_budget() {
    auto p = small();
    p.machine.clearance = .25;
    p.fixtures.push_back({"Threshold plane", box_mesh(Vec3(0, 0, -.85), Vec3(2, 2, 1)), false});
    JointState a, b;
    a.xyz = Vec3(-.5, 0, 0);
    b.xyz = Vec3(.5, 0, 0);
    CollisionScene scene(p);
    CollisionReport r;
    r.status = Status::Clear;
    QVERIFY(!scene.check_interval(a, b, false, a.xyz, b.xyz, 1, 0, 1, r));
    QCOMPARE(r.status, Status::Unresolved);
    QVERIFY(r.checkedIntervals <= 2048);
    QVERIFY(!r.events.empty());
}
QTEST_GUILESS_MAIN(CoreTests)
#include "core_tests.moc"
