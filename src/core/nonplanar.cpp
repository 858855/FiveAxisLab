#include "nonplanar.h"
#include "geometry.h"
#include <Eigen/LU>
#include <algorithm>
#include <map>
#include <stdexcept>
namespace fiveaxis {
std::vector<double> conformal_field(const Project &p) {
    struct Triangle {Vec3 origin;Eigen::Matrix2d inverse;Vec3 b,c;Eigen::AlignedBox2d bounds;};
    std::vector<Triangle> roof;
    for(auto face:p.part.triangles) {
        Vec3 a=p.part.vertices[face[0]],b=p.part.vertices[face[1]],c=p.part.vertices[face[2]];
        Eigen::Matrix2d xy;xy.col(0)=(b-a).head<2>();xy.col(1)=(c-a).head<2>();
        if(std::abs(xy.determinant())<1e-12)continue;
        Triangle t{a,xy.inverse(),b,c,{}};t.bounds.extend(a.head<2>());t.bounds.extend(b.head<2>());t.bounds.extend(c.head<2>());roof.push_back(t);
    }
    double requested=p.part.bounds().min().z()+p.process.nonPlanarBase,anchor=requested,high=p.part.bounds().max().z();
    // Keep every cell intersecting the requested base entirely in the planar field.
    // A coarse mesh can conservatively extend the flat region beyond the requested height.
    for(const auto &cell:p.volume.cells) {
        double low=1e100,top=-1e100;for(int node:cell.nodes){low=std::min(low,p.volume.vertices[node].z());top=std::max(top,p.volume.vertices[node].z());}
        if(low<=requested+1e-8)anchor=std::max(anchor,top);
    }
    std::vector<double> values;values.reserve(p.volume.vertices.size());
    for(const auto &v:p.volume.vertices) {
        if(v.z()<=anchor || anchor>=high-1e-7){values.push_back(v.z());continue;}
        double top=-1e100;
        for(const auto &t:roof) {
            if(t.bounds.exteriorDistance(v.head<2>())>1e-7)continue;
            Eigen::Vector2d w=t.inverse*(v-t.origin).head<2>();
            if(w.minCoeff()<-1e-7 || w.sum()>1+1e-7)continue;
            top=std::max(top,t.origin.z()*(1-w.sum())+t.b.z()*w.x()+t.c.z()*w.y());
        }
        if(top<=anchor+1e-7 || top<v.z()-1e-6)throw std::runtime_error("Cannot establish the model top height for non-planar slicing: XYZ non-planar layers need one top surface above every point, so overhangs or undercuts above the planar base are not supported. Use planar slicing, raise the planar base height, or a rotary mode.");
        values.push_back(anchor+(v.z()-anchor)*(high-anchor)/(top-anchor));
    }
    for(const auto &cell:p.volume.cells) {
        Eigen::Matrix4d m;Eigen::Vector4d f;
        for(int i=0;i<4;++i){m.row(i)<<1,p.volume.vertices[cell.nodes[i]].transpose();f[i]=values[cell.nodes[i]];}
        if(std::abs(m.determinant())<1e-12)throw std::runtime_error("Degenerate non-planar volume cell");
        Vec3 gradient=m.inverse().bottomRows<3>()*f;
        if(!gradient.allFinite() || gradient.z()<=1e-6)throw std::runtime_error("Non-planar field reverses build progression; use planar slicing or decompose the part");
    }
    return values;
}
int cartesian_motion_violation(const Project &p) {
    if(p.machine.rotary_axes())return -1;
    if(p.process.profileNormal)return 0;
    for(size_t i=0;i<p.path.size();++i) {
        const auto &s=p.path[i];
        if(!in_limits(p.machine,p.active_toolhead(),s.joint) || (part_direction(p.machine,p.active_toolhead(),s.joint)-Vec3::UnitZ()).norm()>1e-6 || (s.direction-Vec3::UnitZ()).norm()>1e-6)return int(i);
        if(s.surfaceNormal && (!s.surfaceNormal->allFinite() || std::abs(s.surfaceNormal->norm()-1)>1e-6))return int(i);
        if(s.deltaE>0 && s.role!=Role::Support) {
            if(p.process.nonPlanar3Axis && !s.surfaceNormal)return int(i);
            if(surface_normal(s).z()<std::cos(p.process.nonPlanarSlope)-1e-6)return int(i);
        }
    }
    return -1;
}
Project nonplanar_cartesian_project() {
    Project p;p.machine.layout=Layout::Cartesian3Axis;p.machine.controller=Controller::GenericMarlin;
    p.useDummyToolhead=true;p.dummyToolhead=make_dummy_toolhead(.4);
    p.process.nonPlanar3Axis=true;p.process.curved=false;p.process.layerHeight=.3;p.process.width=.45;p.process.infill=.25;p.process.preciseZ=true;
    p.process.supports=false;p.process.supportMode=SupportMode::None;p.part.name="Three-axis conformal roof test";
    constexpr int count=10,levels=6;auto id=[](int x,int y,int z){return (z*(count+1)+y)*(count+1)+x;};
    for(int z=0;z<=levels;++z)for(int y=0;y<=count;++y)for(int x=0;x<=count;++x) {
        double a=-12+24.0*x/count,b=-12+24.0*y/count,top=7+2*std::cos(pi*a/24)*std::cos(pi*b/24);
        p.volume.vertices.emplace_back(a,b,top*z/levels);
    }
    for(int z=0;z<levels;++z)for(int y=0;y<count;++y)for(int x=0;x<count;++x) {
        int a=id(x,y,z),b=id(x+1,y,z),c=id(x+1,y+1,z),d=id(x,y+1,z),e=id(x,y,z+1),f=id(x+1,y,z+1),g=id(x+1,y+1,z+1),h=id(x,y+1,z+1);
        for(auto nodes:std::vector<std::vector<int>>{{a,b,c,g},{a,c,d,g},{a,d,h,g},{a,h,e,g},{a,e,f,g},{a,f,b,g}})p.volume.cells.push_back({nodes});
    }
    struct Face {Eigen::Vector3i ids;int opposite,count=0;};std::map<std::array<int,3>,Face> faces;
    for(const auto &cell:p.volume.cells)for(int missing=0;missing<4;++missing) {
        Eigen::Vector3i f;int k=0;for(int i=0;i<4;++i)if(i!=missing)f[k++]=cell.nodes[i];
        std::array<int,3> key{f.x(),f.y(),f.z()};std::sort(key.begin(),key.end());auto &entry=faces[key];entry.ids=f;entry.opposite=cell.nodes[missing];++entry.count;
    }
    p.part.vertices=p.volume.vertices;
    for(auto &[key,entry]:faces)if(entry.count==1) {
        auto f=entry.ids;Vec3 a=p.part.vertices[f.x()],b=p.part.vertices[f.y()],c=p.part.vertices[f.z()];
        if((b-a).cross(c-a).dot(p.part.vertices[entry.opposite]-a)>0)std::swap(f.y(),f.z());p.part.triangles.push_back(f);
    }
    p.fixtures.push_back({"Bed",box_mesh(Vec3(0,0,-2),Vec3(100,100,4)),true});return p;
}
}
