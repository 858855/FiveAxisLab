#include "process.h"
#include "geometry.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QCryptographicHash>
#include <QSet>
#include <cmath>
#include <stdexcept>
namespace fiveaxis {
QString process_fingerprint(const ProcessSettings &s) {
    return QString::fromLatin1(QCryptographicHash::hash(QJsonDocument(process_json(s)).toJson(QJsonDocument::Compact),QCryptographicHash::Sha256).toHex());
}
QJsonObject process_json(const ProcessSettings &s) {
    QJsonObject j;
#define FIELD(name) j[#name] = s.name
    FIELD(layerHeight); FIELD(width); FIELD(infill); FIELD(speed); FIELD(directionCone); FIELD(maximumTilt);
    FIELD(curved); FIELD(supports); FIELD(wallCount); FIELD(maximumBridge); FIELD(nozzleDiameter);
    FIELD(firstLayerHeight); FIELD(firstLayerWidth); FIELD(outerWallWidth); FIELD(innerWallWidth);
    FIELD(infillWidth); FIELD(solidWidth); FIELD(topShellLayers); FIELD(bottomShellLayers); FIELD(outerWallFirst);
    FIELD(infillAngle); FIELD(outerWallSpeed); FIELD(innerWallSpeed); FIELD(infillSpeed); FIELD(solidSpeed);
    FIELD(supportSpeed); FIELD(firstLayerSpeed); FIELD(travelSpeed); FIELD(travelLift);
    FIELD(supportSpacing); FIELD(supportReduction);
    FIELD(topWidth); FIELD(supportWidth); FIELD(resolution); FIELD(contourCompensation); FIELD(holeCompensation); FIELD(elephantFoot);
    FIELD(preciseZ); FIELD(infillFirst); FIELD(alternateExtraWall); FIELD(topThickness); FIELD(bottomThickness);
    FIELD(topDensity); FIELD(bottomDensity); FIELD(firstLayerInfillSpeed); FIELD(topSpeed); FIELD(smallWallSpeed); FIELD(smallWallThreshold);
    FIELD(smallWallAbsoluteSpeed);
    FIELD(slowOverhangs); FIELD(supportAngle); FIELD(supportBedOnly); FIELD(removeSmallOverhangs);
    FIELD(skirtLoops); FIELD(skirtHeight); FIELD(brimType); FIELD(skirtDistance); FIELD(brimWidth); FIELD(brimGap);
#undef FIELD
    j["infillPattern"] = int(s.infillPattern); j["seam"] = int(s.seam); j["supportMode"] = int(s.supportMode);
    j["topPattern"] = int(s.topPattern); j["bottomPattern"] = int(s.bottomPattern);
    QJsonArray overhang; for(double v:s.overhangSpeeds) overhang.append(v); j["overhangSpeeds"]=overhang;
    return j;
}
ProcessSettings read_process(const QJsonObject &j) {
    ProcessSettings s;
    auto defaults=process_json(s);
    for(auto it=j.begin();it!=j.end();++it) if(defaults.contains(it.key())) {
        auto expected=defaults[it.key()];
        if((expected.isBool() && !it.value().isBool()) || (expected.isDouble() && !it.value().isDouble()) ||
           (expected.isArray() && !it.value().isArray())) throw std::runtime_error(("Invalid type for process setting: "+it.key()).toStdString());
    }
#define NUMBER(name) s.name = j[#name].toDouble(s.name)
    NUMBER(layerHeight); NUMBER(width); NUMBER(infill); NUMBER(speed); NUMBER(directionCone); NUMBER(maximumTilt);
    NUMBER(maximumBridge); NUMBER(nozzleDiameter); NUMBER(firstLayerHeight); NUMBER(firstLayerWidth);
    NUMBER(outerWallWidth); NUMBER(innerWallWidth); NUMBER(infillWidth); NUMBER(solidWidth); NUMBER(infillAngle);
    NUMBER(outerWallSpeed); NUMBER(innerWallSpeed); NUMBER(infillSpeed); NUMBER(solidSpeed); NUMBER(supportSpeed);
    NUMBER(firstLayerSpeed); NUMBER(travelSpeed); NUMBER(travelLift); NUMBER(supportSpacing); NUMBER(supportReduction);
    NUMBER(topWidth); NUMBER(supportWidth); NUMBER(resolution); NUMBER(contourCompensation); NUMBER(holeCompensation); NUMBER(elephantFoot);
    NUMBER(topThickness); NUMBER(bottomThickness); NUMBER(topDensity); NUMBER(bottomDensity);
    NUMBER(firstLayerInfillSpeed); NUMBER(topSpeed); NUMBER(smallWallSpeed); NUMBER(smallWallThreshold);
    NUMBER(smallWallAbsoluteSpeed);
    NUMBER(supportAngle); NUMBER(skirtDistance); NUMBER(brimWidth); NUMBER(brimGap);
#undef NUMBER
    s.curved = j["curved"].toBool(s.curved); s.supports = j["supports"].toBool(s.supports);
    s.wallCount = j["wallCount"].toInt(s.wallCount);
    s.topShellLayers = j["topShellLayers"].toInt(s.topShellLayers);
    s.bottomShellLayers = j["bottomShellLayers"].toInt(s.bottomShellLayers);
    s.outerWallFirst = j["outerWallFirst"].toBool(s.outerWallFirst);
    s.infillPattern = InfillPattern(j["infillPattern"].toInt());
    s.seam = SeamPosition(j["seam"].toInt());
    s.supportMode = SupportMode(j["supportMode"].toInt(s.supports ? 0 : 2));
    s.topPattern=SurfacePattern(j["topPattern"].toInt(1)); s.bottomPattern=SurfacePattern(j["bottomPattern"].toInt(1));
    s.preciseZ=j["preciseZ"].toBool(); s.infillFirst=j["infillFirst"].toBool(); s.alternateExtraWall=j["alternateExtraWall"].toBool();
    s.slowOverhangs=j["slowOverhangs"].toBool(); s.supportBedOnly=j["supportBedOnly"].toBool(); s.removeSmallOverhangs=j["removeSmallOverhangs"].toBool();
    s.skirtLoops=j["skirtLoops"].toInt(); s.skirtHeight=j["skirtHeight"].toInt(1); s.brimType=j["brimType"].toInt();
    if(j.contains("overhangSpeeds")) { auto a=j["overhangSpeeds"].toArray(); if(a.size()!=5) throw std::runtime_error("Expected five overhang speeds"); for(int i=0;i<5;++i) s.overhangSpeeds[i]=a[i].toDouble(-1); }
    validate_process(s);
    return s;
}
double extrusion_height(const ProcessSettings &s, int layer) {
    return layer == 0 && s.firstLayerHeight > 0 ? s.firstLayerHeight : s.layerHeight;
}
double extrusion_width(const ProcessSettings &s, Role role, int layer) {
    if (layer == 0 && s.firstLayerWidth > 0) return s.firstLayerWidth;
    double value = role == Role::OuterWall ? s.outerWallWidth : role == Role::Wall ? s.innerWallWidth :
                   role == Role::Infill ? s.infillWidth : role == Role::TopSurface ? s.topWidth :
                   role == Role::Support ? s.supportWidth : (role == Role::SolidInfill || role == Role::BottomSurface) ? s.solidWidth : 0;
    return value > 0 ? value : s.width;
}
double process_speed(const ProcessSettings &s, Role role, int layer) {
    if (role == Role::Travel) return s.travelSpeed;
    if(layer==0 && s.firstLayerInfillSpeed>0 && (role==Role::Infill || role==Role::SolidInfill || role==Role::TopSurface || role==Role::BottomSurface)) return s.firstLayerInfillSpeed;
    if (layer == 0 && s.firstLayerSpeed > 0) return s.firstLayerSpeed;
    double value = role == Role::OuterWall ? s.outerWallSpeed : role == Role::Wall ? s.innerWallSpeed :
                   role == Role::Infill ? s.infillSpeed : role == Role::TopSurface ? s.topSpeed : (role == Role::SolidInfill || role == Role::BottomSurface) ? s.solidSpeed :
                   role == Role::Support ? s.supportSpeed : 0;
    return value > 0 ? value : s.speed;
}
int effective_walls(const ProcessSettings &s,int layer) { return s.wallCount+(s.alternateExtraWall && layer%2==1 ? 1 : 0); }
void validate_process(const ProcessSettings &s) {
    for (auto value : process_json(s))
        if (value.isDouble() && !std::isfinite(value.toDouble())) throw std::runtime_error("Non-finite process setting");
    if (s.layerHeight <= 0 || s.nozzleDiameter <= 0 || s.width < s.layerHeight || s.infill < 0 || s.infill > 1 ||
        s.speed <= 0 || s.travelSpeed <= 0 || s.travelLift < 0 || s.wallCount < 0 || s.wallCount > 50 ||
        s.topShellLayers < 0 || s.topShellLayers > 100 || s.bottomShellLayers < 0 || s.bottomShellLayers > 100 ||
        s.firstLayerHeight < 0 || s.supportSpacing < 0 || s.supportReduction < 1 || s.maximumBridge < 0 ||
        s.directionCone < 0 || s.directionCone > pi/2 || s.maximumTilt < 0 || s.maximumTilt >= pi/2 ||
        int(s.infillPattern) < 0 || int(s.infillPattern) > 2 || int(s.seam) < 0 || int(s.seam) > 4 ||
        int(s.supportMode) < 0 || int(s.supportMode) > 2)
        throw std::runtime_error("Invalid process settings");
    if(s.resolution<0 || std::abs(s.contourCompensation)>2 || std::abs(s.holeCompensation)>2 || s.elephantFoot<0 || s.elephantFoot>2 ||
       s.topThickness<0 || s.bottomThickness<0 || s.topDensity<0 || s.topDensity>1 || s.bottomDensity<0 || s.bottomDensity>1 ||
       int(s.topPattern)<0 || int(s.topPattern)>2 || int(s.bottomPattern)<0 || int(s.bottomPattern)>2 ||
       s.firstLayerInfillSpeed<0 || s.topSpeed<0 || s.smallWallSpeed<=0 || s.smallWallSpeed>1 || s.smallWallThreshold<0 || s.smallWallAbsoluteSpeed<0 ||
       s.supportAngle<0 || s.supportAngle>=pi/2 || s.skirtLoops<0 || s.skirtLoops>50 || s.skirtHeight<1 || s.skirtHeight>100 ||
       s.brimType<0 || s.brimType>4 || s.skirtDistance<0 || s.brimWidth<0 || s.brimWidth>50 || s.brimGap<0)
        throw std::runtime_error("Invalid advanced process setting");
    for(double v:s.overhangSpeeds) if(!std::isfinite(v) || v<0) throw std::runtime_error("Invalid overhang speed");
    for (double v : {s.outerWallSpeed,s.innerWallSpeed,s.infillSpeed,s.solidSpeed,s.supportSpeed,s.firstLayerSpeed})
        if (v < 0) throw std::runtime_error("Print speeds must be nonnegative; zero inherits the default");
    for (Role role : {Role::Wall,Role::OuterWall,Role::Infill,Role::SolidInfill,Role::Support,Role::TopSurface,Role::BottomSurface})
        for (int layer : {0,1})
            if (extrusion_width(s,role,layer) < extrusion_height(s,layer))
                throw std::runtime_error("Extrusion width must be at least its layer height");
    for (double v : {s.firstLayerWidth,s.outerWallWidth,s.innerWallWidth,s.infillWidth,s.solidWidth,s.topWidth,s.supportWidth})
        if (v < 0) throw std::runtime_error("Line widths must be nonnegative; zero inherits the default");
}
ProcessSettings import_orca_process(const QJsonObject &j, const ProcessSettings &base, QStringList &notes) {
    ProcessSettings s = base;
    QSet<QString> used;
    auto text = [&](const QString &key) {
        auto value = j[key]; used.insert(key);
        if (value.isArray()) { auto a=value.toArray(); if (a.size()!=1) throw std::runtime_error("Only one extruder is supported"); value=a[0]; }
        return value.isString() ? value.toString() : value.isBool() ? (value.toBool()?QString("1"):QString("0")) : QString::number(value.toDouble());
    };
    auto number = [&](const QString &key, double &target, bool percent=false) {
        if (!j.contains(key)) return;
        QString v=text(key).trimmed(); bool hasPercent=v.endsWith('%'); if(hasPercent) v.chop(1);
        bool ok=false; double n=v.toDouble(&ok);
        if(!ok || !std::isfinite(n) || (hasPercent && !percent)) throw std::runtime_error(("Unsupported numeric format: "+key).toStdString());
        target=percent?n/100:n;
    };
    number("layer_height",s.layerHeight); number("initial_layer_print_height",s.firstLayerHeight);
    number("line_width",s.width); number("initial_layer_line_width",s.firstLayerWidth);
    number("outer_wall_line_width",s.outerWallWidth); number("inner_wall_line_width",s.innerWallWidth);
    number("sparse_infill_line_width",s.infillWidth); number("internal_solid_infill_line_width",s.solidWidth);
    number("top_surface_line_width",s.topWidth); number("support_line_width",s.supportWidth);
    number("resolution",s.resolution); number("xy_contour_compensation",s.contourCompensation);
    number("xy_hole_compensation",s.holeCompensation); number("elefant_foot_compensation",s.elephantFoot);
    number("top_shell_thickness",s.topThickness); number("bottom_shell_thickness",s.bottomThickness);
    number("top_surface_density",s.topDensity,true); number("bottom_surface_density",s.bottomDensity,true);
    number("sparse_infill_density",s.infill,true); number("nozzle_diameter",s.nozzleDiameter);
    if(s.width==0) s.width=default_line_width(s.nozzleDiameter);
    number("outer_wall_speed",s.outerWallSpeed); number("inner_wall_speed",s.innerWallSpeed);
    number("sparse_infill_speed",s.infillSpeed); number("internal_solid_infill_speed",s.solidSpeed);
    number("support_speed",s.supportSpeed); number("initial_layer_speed",s.firstLayerSpeed);
    number("initial_layer_infill_speed",s.firstLayerInfillSpeed); number("top_surface_speed",s.topSpeed);
    if(j.contains("small_perimeter_speed")) {
        if(text("small_perimeter_speed").endsWith('%')) {number("small_perimeter_speed",s.smallWallSpeed,true);s.smallWallAbsoluteSpeed=0;}
        else number("small_perimeter_speed",s.smallWallAbsoluteSpeed);
    }
    number("small_perimeter_threshold",s.smallWallThreshold);
    number("skirt_distance",s.skirtDistance); number("brim_width",s.brimWidth); number("brim_object_gap",s.brimGap);
    number("travel_speed",s.travelSpeed); number("z_hop",s.travelLift);
    number("support_base_pattern_spacing",s.supportSpacing);
    for(auto pair : {std::make_pair("wall_loops", &s.wallCount),std::make_pair("top_shell_layers", &s.topShellLayers),std::make_pair("bottom_shell_layers", &s.bottomShellLayers)}) {
        if(j.contains(pair.first)) { double n=*pair.second; number(pair.first,n); if(n!=std::floor(n) || n<0 || n>100) throw std::runtime_error("Layer and wall counts must be bounded nonnegative integers"); *pair.second=int(n); }
    }
    for(auto pair:{std::make_pair("skirt_loops",&s.skirtLoops),std::make_pair("skirt_height",&s.skirtHeight)})
        if(j.contains(pair.first)) { double n=*pair.second; number(pair.first,n); if(n!=std::floor(n)) throw std::runtime_error("Expected an integer skirt count"); *pair.second=int(n); }
    auto boolean=[&](const QString &key,bool &target) { if(!j.contains(key)) return; auto v=text(key); if(v!="0" && v!="1") throw std::runtime_error(("Invalid boolean: "+key).toStdString()); target=v=="1"; };
    boolean("precise_z_height",s.preciseZ); boolean("infill_first",s.infillFirst); boolean("alternate_extra_wall",s.alternateExtraWall);
    boolean("slow_down_for_overhang",s.slowOverhangs); boolean("support_on_build_plate_only",s.supportBedOnly); boolean("support_remove_small_overhang",s.removeSmallOverhangs);
    boolean("is_infill_first",s.infillFirst); boolean("enable_overhang_speed",s.slowOverhangs);
    for(int i=0;i<4;++i) number(QString("overhang_%1_4_speed").arg(i+1),s.overhangSpeeds[i]);
    number("overhang_totally_speed",s.overhangSpeeds[4]);
    if(j.contains("support_threshold_angle")) { double degrees=s.supportAngle*180/pi; number("support_threshold_angle",degrees); s.supportAngle=degrees*pi/180; }
    for(auto pair:{std::make_pair("top_surface_pattern",&s.topPattern),std::make_pair("bottom_surface_pattern",&s.bottomPattern)}) if(j.contains(pair.first)) {
        auto v=text(pair.first); if(v=="rectilinear") *pair.second=SurfacePattern::Rectilinear;
        else if(v=="monotonic" || v=="monotonicline") *pair.second=SurfacePattern::Monotonic;
        else if(v=="concentric") *pair.second=SurfacePattern::Concentric;
        else throw std::runtime_error(("Surface pattern is not implemented: "+v).toStdString());
    }
    if(j.contains("brim_type")) { auto v=text("brim_type");
        if(v=="no_brim") s.brimType=0; else if(v=="auto_brim") s.brimType=1; else if(v=="outer_only") s.brimType=2;
        else if(v=="inner_only") s.brimType=3; else if(v=="outer_and_inner") s.brimType=4;
        else throw std::runtime_error("Unsupported brim type"); }
    if(j.contains("infill_direction")) { double degrees=s.infillAngle*180/pi; number("infill_direction",degrees); s.infillAngle=degrees*pi/180; }
    if(j.contains("sparse_infill_pattern")) { auto value=text("sparse_infill_pattern");
        if(value=="rectilinear" || value=="line") s.infillPattern=InfillPattern::Rectilinear;
        else if(value=="grid") s.infillPattern=InfillPattern::Grid;
        else throw std::runtime_error(("Infill pattern is not implemented: "+value).toStdString()); }
    if(j.contains("seam_position")) { auto value=text("seam_position");
        if(value=="aligned") s.seam=SeamPosition::Aligned; else if(value=="back" || value=="rear") s.seam=SeamPosition::Back;
        else if(value=="nearest") s.seam=SeamPosition::Nearest; else if(value=="random") s.seam=SeamPosition::Random;
        else throw std::runtime_error(("Seam position is not implemented: "+value).toStdString()); }
    if(j.contains("wall_sequence")) { auto value=text("wall_sequence");
        if(value=="outer wall/inner wall") s.outerWallFirst=true;
        else if(value=="inner wall/outer wall") s.outerWallFirst=false;
        else throw std::runtime_error("Only outer/inner and inner/outer wall order are supported"); }
    if(j.contains("enable_support")) { auto value=text("enable_support"); if(value!="0" && value!="1") throw std::runtime_error("Invalid enable_support");
        s.supports=value=="1"; s.supportMode=s.supports?SupportMode::Normal:SupportMode::None; }
    for(auto it=j.begin();it!=j.end();++it)
        if(!used.contains(it.key()) && it.key()!="name" && it.key()!="type" && it.key()!="from" && it.key()!="setting_id" && it.key()!="version")
            notes << "Not imported: " + it.key();
    if(j.contains("inherits")) notes << "Preset inheritance is not resolved; supply a flattened preset or set inherited values manually.";
    validate_process(s); return s;
}
} // namespace fiveaxis
