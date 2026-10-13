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
#undef FIELD
    j["infillPattern"] = int(s.infillPattern); j["seam"] = int(s.seam); j["supportMode"] = int(s.supportMode);
    return j;
}
ProcessSettings read_process(const QJsonObject &j) {
    ProcessSettings s;
#define NUMBER(name) s.name = j[#name].toDouble(s.name)
    NUMBER(layerHeight); NUMBER(width); NUMBER(infill); NUMBER(speed); NUMBER(directionCone); NUMBER(maximumTilt);
    NUMBER(maximumBridge); NUMBER(nozzleDiameter); NUMBER(firstLayerHeight); NUMBER(firstLayerWidth);
    NUMBER(outerWallWidth); NUMBER(innerWallWidth); NUMBER(infillWidth); NUMBER(solidWidth); NUMBER(infillAngle);
    NUMBER(outerWallSpeed); NUMBER(innerWallSpeed); NUMBER(infillSpeed); NUMBER(solidSpeed); NUMBER(supportSpeed);
    NUMBER(firstLayerSpeed); NUMBER(travelSpeed); NUMBER(travelLift); NUMBER(supportSpacing); NUMBER(supportReduction);
#undef NUMBER
    s.curved = j["curved"].toBool(s.curved); s.supports = j["supports"].toBool(s.supports);
    s.wallCount = j["wallCount"].toInt(s.wallCount);
    s.topShellLayers = j["topShellLayers"].toInt(s.topShellLayers);
    s.bottomShellLayers = j["bottomShellLayers"].toInt(s.bottomShellLayers);
    s.outerWallFirst = j["outerWallFirst"].toBool(s.outerWallFirst);
    s.infillPattern = InfillPattern(j["infillPattern"].toInt());
    s.seam = SeamPosition(j["seam"].toInt());
    s.supportMode = SupportMode(j["supportMode"].toInt(s.supports ? 0 : 2));
    validate_process(s);
    return s;
}
double extrusion_height(const ProcessSettings &s, int layer) {
    return layer == 0 && s.firstLayerHeight > 0 ? s.firstLayerHeight : s.layerHeight;
}
double extrusion_width(const ProcessSettings &s, Role role, int layer) {
    if (layer == 0 && s.firstLayerWidth > 0) return s.firstLayerWidth;
    double value = role == Role::OuterWall ? s.outerWallWidth : role == Role::Wall ? s.innerWallWidth :
                   role == Role::Infill ? s.infillWidth : role == Role::SolidInfill ? s.solidWidth : 0;
    return value > 0 ? value : s.width;
}
double process_speed(const ProcessSettings &s, Role role, int layer) {
    if (role == Role::Travel) return s.travelSpeed;
    if (layer == 0 && s.firstLayerSpeed > 0) return s.firstLayerSpeed;
    double value = role == Role::OuterWall ? s.outerWallSpeed : role == Role::Wall ? s.innerWallSpeed :
                   role == Role::Infill ? s.infillSpeed : role == Role::SolidInfill ? s.solidSpeed :
                   role == Role::Support ? s.supportSpeed : 0;
    return value > 0 ? value : s.speed;
}
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
    for (double v : {s.outerWallSpeed,s.innerWallSpeed,s.infillSpeed,s.solidSpeed,s.supportSpeed,s.firstLayerSpeed})
        if (v < 0) throw std::runtime_error("Print speeds must be nonnegative; zero inherits the default");
    for (Role role : {Role::Wall,Role::OuterWall,Role::Infill,Role::SolidInfill,Role::Support})
        for (int layer : {0,1})
            if (extrusion_width(s,role,layer) < extrusion_height(s,layer))
                throw std::runtime_error("Extrusion width must be at least its layer height");
    for (double v : {s.firstLayerWidth,s.outerWallWidth,s.innerWallWidth,s.infillWidth,s.solidWidth})
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
    number("sparse_infill_density",s.infill,true); number("nozzle_diameter",s.nozzleDiameter);
    if(s.width==0) s.width=default_line_width(s.nozzleDiameter);
    number("outer_wall_speed",s.outerWallSpeed); number("inner_wall_speed",s.innerWallSpeed);
    number("sparse_infill_speed",s.infillSpeed); number("internal_solid_infill_speed",s.solidSpeed);
    number("support_speed",s.supportSpeed); number("initial_layer_speed",s.firstLayerSpeed);
    number("travel_speed",s.travelSpeed); number("z_hop",s.travelLift);
    number("support_base_pattern_spacing",s.supportSpacing);
    for(auto pair : {std::make_pair("wall_loops", &s.wallCount),std::make_pair("top_shell_layers", &s.topShellLayers),std::make_pair("bottom_shell_layers", &s.bottomShellLayers)}) {
        if(j.contains(pair.first)) { double n=*pair.second; number(pair.first,n); if(n!=std::floor(n) || n<0 || n>100) throw std::runtime_error("Layer and wall counts must be bounded nonnegative integers"); *pair.second=int(n); }
    }
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
