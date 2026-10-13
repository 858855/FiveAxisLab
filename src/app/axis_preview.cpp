#include "axis_preview.h"
#include "core/motion_preview.h"
#include <QCheckBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
using namespace fiveaxis;
AxisPreview::AxisPreview(QWidget *parent):QGroupBox("Axis behavior",parent) {
    setObjectName("axisPreview");auto column=new QVBoxLayout(this);column->setContentsMargins(6,12,6,5);column->setSpacing(4);
    source=new QComboBox;source->setObjectName("axisPreviewSource");source->addItems({"Follow toolpath","Manual tilt / roll","Animated tilt / roll"});column->addWidget(source);
    auto form=new QFormLayout;form->setContentsMargins(0,0,0,0);form->setSpacing(4);column->addLayout(form);
    auto control=[&](const QString &name,QDoubleSpinBox *&spin,QSlider *&slider,QLabel *&label) {
        auto row=new QWidget;auto horizontal=new QHBoxLayout(row);horizontal->setContentsMargins(0,0,0,0);
        slider=new QSlider(Qt::Horizontal);slider->setObjectName(name+"Slider");spin=new QDoubleSpinBox;spin->setObjectName(name);spin->setDecimals(1);spin->setSuffix("°");spin->setKeyboardTracking(false);spin->setFixedWidth(76);
        horizontal->addWidget(slider,1);horizontal->addWidget(spin);label=new QLabel;form->addRow(label,row);
        connect(slider,&QSlider::valueChanged,this,[this,spin](int value){if(!updating)spin->setValue(value/10.0);});
        connect(spin,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this,slider](double value){
            QSignalBlocker guard(slider);slider->setValue(int(std::round(value*10)));
            if(!updating){if(source->currentIndex()!=1){QSignalBlocker sourceGuard(source);source->setCurrentIndex(1);change_source(1);}else apply_pose();}
        });
    };
    control("previewAxisA",a,sliderA,labelA);control("previewAxisB",b,sliderB,labelB);
    auto buttons=new QHBoxLayout;animate=new QPushButton("Animate");animate->setCheckable(true);animate->setObjectName("animateAxes");
    auto reset=new QPushButton("Zero axes");reset->setObjectName("resetPreviewAxes");buttons->addWidget(animate);buttons->addWidget(reset);column->addLayout(buttons);
    status=new QLabel;status->setObjectName("axisPreviewStatus");status->setWordWrap(true);column->addWidget(status);
    connect(source,&QComboBox::currentIndexChanged,this,&AxisPreview::change_source);
    connect(animate,&QPushButton::toggled,this,[this](bool enabled){
        if(enabled){if(source->currentIndex()!=2)source->setCurrentIndex(2);elapsed.restart();timer.start();animate->setText("Pause axes");}
        else {timer.stop();animate->setText("Animate");}
    });
    connect(reset,&QPushButton::clicked,this,[this]{pause();QSignalBlocker guard(source);source->setCurrentIndex(1);set_angles(0,0);apply_pose();});
    timer.setInterval(33);connect(&timer,&QTimer::timeout,this,&AxisPreview::tick);
}
void AxisPreview::set_project(const Project *p,const JointState &q) {
    pause();project=p;updating=true;
    if(p){
        labelA->setText(p->machine.layout==Layout::RotaryBed?"A · tilt":"A · roll");labelB->setText(p->machine.axis2()+(p->machine.layout==Layout::RotaryBed?" · rotation":" · tilt"));
        a->setRange(p->machine.minA*180/pi,p->machine.maxA*180/pi);b->setRange(p->machine.minB*180/pi,p->machine.maxB*180/pi);
        sliderA->setRange(int(std::ceil(p->machine.minA*1800/pi)),int(std::floor(p->machine.maxA*1800/pi)));
        sliderB->setRange(int(std::ceil(p->machine.minB*1800/pi)),int(std::floor(p->machine.maxB*1800/pi)));
    }
    updating=false;follow_motion(q);
}
void AxisPreview::set_angles(double first,double second) {
    updating=true; a->setValue(first*180/pi);b->setValue(second*180/pi);
    sliderA->setValue(int(std::round(a->value()*10)));sliderB->setValue(int(std::round(b->value()*10)));updating=false;
}
void AxisPreview::follow_motion(const JointState &q) {
    pause();QSignalBlocker guard(source);source->setCurrentIndex(0);recorded=displayed=q;
    if(project)anchor=part_position(project->machine,q);
    set_angles(q.a,q.b);status->setText("Actual planned / replayed joint angles. Drag an axis to inspect a pose.");
    emit pose_changed(q,false);
}
void AxisPreview::pause(){animate->setChecked(false);timer.stop();}
void AxisPreview::toggle_animation(){animate->setChecked(!animate->isChecked());}
void AxisPreview::change_source(int index) {
    if(!project)return;
    if(index==0){follow_motion(recorded);return;}
    if(index==1){pause();apply_pose();return;}
    phase=0;apply_pose();animate->setChecked(true);
}
void AxisPreview::apply_pose() {
    if(!project)return;
    auto q=rotary_preview_pose(project->machine,anchor,std::clamp(a->value()*pi/180,project->machine.minA,project->machine.maxA),std::clamp(b->value()*pi/180,project->machine.minB,project->machine.maxB));
    if(in_limits(project->machine,project->active_toolhead(),q)){
        displayed=q;status->setText("Pose preview only · no extrusion or collision validation. TCP follows the same point on the workpiece.");
    }else{
        set_angles(displayed.a,displayed.b);status->setText("Carriage limit reached; last reachable pose retained. Pose preview is not validated motion.");
    }
    emit pose_changed(displayed,true);
}
void AxisPreview::tick() {
    if(!project)return;
    const auto &m=project->machine;
    double amplitudeA=std::min(40*pi/180,(m.maxA-m.minA)/2),amplitudeB=std::min((m.layout==Layout::RotaryBed?90:45)*pi/180,(m.maxB-m.minB)/2);
    double largest=std::max({amplitudeA,amplitudeB,1e-8});
    double omega=std::min({.6,.7*m.angularSpeed/largest,.7*std::sqrt(m.angularAccel/largest)});
    phase+=elapsed.restart()/1000.0*omega;
    double centerA=std::clamp(0.0,m.minA+amplitudeA,m.maxA-amplitudeA),centerB=std::clamp(0.0,m.minB+amplitudeB,m.maxB-amplitudeB);
    set_angles(centerA+amplitudeA*std::sin(phase),centerB+amplitudeB*std::sin(phase*.77));apply_pose();
}
