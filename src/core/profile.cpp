#include "profile.h"
#include "geometry.h"
#include "process.h"
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <algorithm>
#include <stdexcept>
namespace fiveaxis {
void validate_profile_guide(const std::vector<ProfileStation> &guide) {
    if(guide.size()<2)throw std::runtime_error("Profile-normal slicing requires a centerline guide with at least two stations");
    for(size_t i=0;i<guide.size();++i) {
        const auto &s=guide[i];
        if(!s.center.allFinite() || !s.normal.allFinite() || std::abs(s.normal.norm()-1)>1e-6)
            throw std::runtime_error("Invalid profile center or unit normal");
        if(i) {
            Vec3 step=s.center-guide[i-1].center;
            if(step.norm()<1e-7 || step.normalized().dot(s.normal)<=0 || step.normalized().dot(guide[i-1].normal)<=0)
                throw std::runtime_error("Profile stations must progress along their normals without duplicates or reversals");
        }
    }
}
double profile_length(const std::vector<ProfileStation> &guide) {
    double length=0;for(size_t i=1;i<guide.size();++i)length+=(guide[i].center-guide[i-1].center).norm();return length;
}
ProfileStation profile_section(const std::vector<ProfileStation> &guide,double distance) {
    if(guide.empty())throw std::runtime_error("Missing profile guide");
    distance=std::max(0.0,distance);
    for(size_t i=1;i<guide.size();++i) {
        double length=(guide[i].center-guide[i-1].center).norm();
        if(distance<=length) {
            double u=distance/length;return {guide[i-1].center*(1-u)+guide[i].center*u,(guide[i-1].normal*(1-u)+guide[i].normal*u).normalized()};
        }
        distance-=length;
    }
    return guide.back();
}
ProfileStation profile_layer(const Project &p,int layer) {
    return profile_section(p.profileGuide,std::min(profile_length(p.profileGuide),extrusion_height(p.process,0)+std::max(0,layer)*p.process.layerHeight));
}
std::vector<ProfileStation> import_profile_guide(const QString &file,const Transform &placement) {
    QFile input(file);if(!input.open(QIODevice::ReadOnly))throw std::runtime_error("Cannot open profile guide JSON");
    QJsonParseError error;auto doc=QJsonDocument::fromJson(input.readAll(),&error);
    if(error.error!=QJsonParseError::NoError || !doc.isObject() || !doc.object()["stations"].isArray())throw std::runtime_error("Expected JSON object with stations in source-model millimetres");
    if(doc.object().contains("units") && doc.object()["units"].toString()!="mm")throw std::runtime_error("Profile guide units must be mm");
    auto vector=[](QJsonValue value){auto a=value.toArray();if(a.size()!=3 || !a[0].isDouble() || !a[1].isDouble() || !a[2].isDouble())throw std::runtime_error("Expected three numeric profile coordinates");return Vec3(a[0].toDouble(),a[1].toDouble(),a[2].toDouble());};
    std::vector<ProfileStation> result;
    for(auto entry:doc.object()["stations"].toArray()) {
        auto obj=entry.toObject();Vec3 n=vector(obj["normal"]);if(!n.allFinite() || n.norm()<1e-8)throw std::runtime_error("Invalid profile normal");
        result.push_back({placement*vector(obj["center"]),(placement.linear()*n).normalized()});
    }
    validate_profile_guide(result);return result;
}
int profile_motion_violation(const Project &p) {
    if(!p.process.profileNormal)return -1;
    validate_profile_guide(p.profileGuide);
    for(size_t i=1;i<p.path.size();++i) {
        const auto &s=p.path[i];if(s.deltaE<=0 || s.role==Role::Support)continue;
        const auto &previous=p.path[i-1];
        if(std::abs(previous.joint.a-s.joint.a)>1e-8 || std::abs(previous.joint.b-s.joint.b)>1e-8)return int(i);
        auto section=profile_layer(p,s.layer);
        for(double u:{0.0,.5,1.0}) {
            auto q=interpolate_motion(p.machine,p.active_toolhead(),p.path[i-1].joint,s.joint,u);
            if(!q.xyz.allFinite() || !std::isfinite(q.a) || !std::isfinite(q.b) || (part_direction(p.machine,p.active_toolhead(),q)-section.normal).norm()>1e-6 ||
               std::abs((part_position(p.machine,q)-section.center).dot(section.normal))>p.machine.chordTolerance+1e-6)
                return int(i);
        }
    }
    return -1;
}
Project spiral_profile_project(Layout layout) {
    Project p;p.machine.layout=layout;p.machine.minB=layout==Layout::RotaryBed?-pi:-75*pi/180;p.machine.maxB=-p.machine.minB;
    p.useDummyToolhead=true;p.dummyToolhead=make_dummy_toolhead(.6);p.process.nozzleDiameter=.6;
    p.process.profileNormal=true;p.process.curved=false;p.process.layerHeight=.5;p.process.width=.65;p.process.infill=.2;p.process.wallCount=2;
    p.process.supports=false;p.process.supportMode=SupportMode::None;p.process.preciseZ=true;
    p.part.name="Spiral five-axis profile test";
    constexpr int rings=96,sides=20;constexpr double radius=3;
    Vec3 side=Vec3::UnitX();
    for(int i=0;i<=rings;++i) {
        double t=double(i)/rings,phase=2*pi*t*t*(3-2*t),rate=12*pi*t*(1-t);
        Vec3 center(9*(1-std::cos(phase))-9,9*std::sin(phase),60*t),normal(9*std::sin(phase)*rate,9*std::cos(phase)*rate,60);normal.normalize();
        p.profileGuide.push_back({center,normal});side-=normal*side.dot(normal);side.normalize();Vec3 other=normal.cross(side);
        for(int j=0;j<sides;++j) {double angle=2*pi*j/sides;p.part.vertices.push_back(center+radius*(side*std::cos(angle)+other*std::sin(angle)));}
    }
    for(int i=0;i<rings;++i)for(int j=0;j<sides;++j) {
        int a=i*sides+j,b=i*sides+(j+1)%sides,c=b+sides,d=a+sides;p.part.triangles.emplace_back(a,b,c);p.part.triangles.emplace_back(a,c,d);
    }
    int bottom=int(p.part.vertices.size());p.part.vertices.push_back(p.profileGuide.front().center);int top=int(p.part.vertices.size());p.part.vertices.push_back(p.profileGuide.back().center);
    for(int j=0;j<sides;++j){p.part.triangles.emplace_back(bottom,(j+1)%sides,j);p.part.triangles.emplace_back(top,rings*sides+j,rings*sides+(j+1)%sides);}
    // The synthetic sweep supplies a conforming volume over the same triangulated boundary.
    p.volume.vertices=p.part.vertices;std::vector<int> centers(rings+1);centers.front()=bottom;centers.back()=top;
    for(int i=1;i<rings;++i){centers[i]=int(p.volume.vertices.size());p.volume.vertices.push_back(p.profileGuide[i].center);}
    for(int i=0;i<rings;++i)for(int j=0;j<sides;++j) {
        int a=centers[i],b=i*sides+j,c=i*sides+(j+1)%sides,A=centers[i+1],B=b+sides,C=c+sides;
        p.volume.cells.push_back({{a,b,c,C}});p.volume.cells.push_back({{a,b,B,C}});p.volume.cells.push_back({{a,A,B,C}});
    }
    p.fixtures.push_back({"Bed",box_mesh(Vec3(0,0,-2),Vec3(100,100,4)),true});
    return p;
}
}
