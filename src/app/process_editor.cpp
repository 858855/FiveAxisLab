#include "process_editor.h"
#include "core/process.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QLabel>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <stdexcept>
using namespace fiveaxis;

ProcessEditor::ProcessEditor(QWidget *parent):QWidget(parent) {
    setObjectName("processEditor"); auto outer=new QVBoxLayout(this); outer->setContentsMargins(0,0,0,0); outer->setSpacing(5);
    auto options=new QHBoxLayout; options->addWidget(new QLabel("Process · Global")); options->addStretch();
    advanced=new QCheckBox("Advanced"); advanced->setObjectName("advancedSettings"); advanced->setChecked(true); options->addWidget(advanced); outer->addLayout(options);
    search=new QLineEdit; search->setPlaceholderText("Search settings…"); search->setObjectName("settingsSearch"); outer->addWidget(search);
    pages=new QTabWidget; pages->setObjectName("processTabs"); outer->addWidget(pages,1);
    QList<QVBoxLayout*> columns;
    for(const auto &title:{"Quality","Strength","Speed","Support","Other"}) {
        auto scroll=new QScrollArea; scroll->setWidgetResizable(true); scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        auto panel=new QWidget; auto column=new QVBoxLayout(panel); column->setContentsMargins(6,6,6,6); column->setSpacing(6);
        scroll->setWidget(panel); pages->addTab(scroll,title); columns.push_back(column);
    }
    QVBoxLayout *group=nullptr;
    auto section=[&](int page,const QString &title) { auto box=new QGroupBox(title); group=new QVBoxLayout(box); group->setSpacing(4); group->setContentsMargins(6,12,6,6); columns[page]->addWidget(box); };
    auto row=[&](const QString &key,const QString &label,QWidget *input,double scale,bool expert) {
        auto container=new QWidget; auto layout=new QHBoxLayout(container); layout->setContentsMargins(0,0,0,0); layout->setSpacing(6);
        auto name=new QLabel(label); name->setWordWrap(true); name->setMinimumWidth(140); layout->addWidget(name,1); input->setMaximumWidth(126); layout->addWidget(input);
        input->setObjectName(key); group->addWidget(container); bindings.insert(key,{input,container,scale,expert,label});
    };
    auto number=[&](const QString &key,const QString &label,double min,double max,const QString &unit=QString{},double scale=1,bool expert=false) {
        auto input=new QDoubleSpinBox; input->setDecimals(unit=="%"?1:unit=="°"?1:3); input->setRange(min,max); input->setSuffix(unit.isEmpty()?QString{}:" "+unit); input->setKeyboardTracking(false);
        row(key,label,input,scale,expert); connect(input,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this]{changed();});
        if(min==0) input->setToolTip("Zero inherits the default where applicable. Geometry and machine limits still apply.");
    };
    auto count=[&](const QString &key,const QString &label,int min,int max,bool expert=false) {
        auto input=new QSpinBox; input->setRange(min,max); input->setKeyboardTracking(false); row(key,label,input,1,expert); connect(input,qOverload<int>(&QSpinBox::valueChanged),this,[this]{changed();});
    };
    auto check=[&](const QString &key,const QString &label,bool expert=false) {
        auto input=new QCheckBox; row(key,label,input,1,expert); connect(input,&QCheckBox::toggled,this,[this]{changed();});
    };
    auto choice=[&](const QString &key,const QString &label,const QStringList &items,bool expert=false) {
        auto input=new QComboBox; input->addItems(items); row(key,label,input,1,expert); connect(input,&QComboBox::currentIndexChanged,this,[this]{changed();});
    };
    auto unavailable=[&](const QString &key,const QString &label,const QString &reason,const QString &value=QString{}) {
        auto input=new QLineEdit(value.isEmpty()?"Unavailable":value); input->setEnabled(false); row(key,label,input,1,true); input->parentWidget()->setToolTip(reason+" · Not applied to generation.");
    };
    section(0,"Layer height"); number("layerHeight","Layer height",.01,2,"mm"); number("firstLayerHeight","First layer height",0,2,"mm");
    unavailable("mixedColorLayer","Mixed-color sublayer","The research preset supports one extruder");
    section(0,"Line width"); number("nozzleDiameter","Nozzle diameter",.1,3,"mm");
    for(const auto &item:QList<QPair<QString,QString>>{{"width","Default"},{"firstLayerWidth","First layer"},{"outerWallWidth","Outer wall"},{"innerWallWidth","Inner wall"},{"topWidth","Top surface"},{"infillWidth","Sparse infill"},{"solidWidth","Internal solid infill"},{"supportWidth","Support"}})
        number(item.first,item.second,item.first=="width"?.01:0,5,"mm");
    section(0,"Seam"); choice("seam","Seam position",{"Lowest stress","Aligned","Back","Nearest","Random"});
    for(const auto &item:QList<QPair<QString,QString>>{{"seamAvoidOverhang","Seam away from overhangs"},{"scarfSmart","Smart scarf application"},{"scarfAngle","Scarf angle threshold"},{"scarfWhole","Scarf entire wall"},{"scarfSteps","Scarf steps"},{"scarfInner","Scarf inner walls"},{"scarfOverride","Override filament scarf"},{"roleWipeSpeed","Role-based wipe speed"}})
        unavailable(item.first,item.second,"Scarf and wipe trajectories require the full Orca extrusion planner");
    section(0,"Precision"); number("resolution","Path resolution",0,.2,"mm",1,true);
    unavailable("sliceClosingRadius","Slice gap closing radius","Volumetric meshing currently requires a closed input solid");
    unavailable("arcFitting","Arc fitting","Coordinated five-axis export currently uses validated G1 intervals");
    number("holeCompensation","X-Y hole compensation",-2,2,"mm",1,true); number("contourCompensation","X-Y contour compensation",-2,2,"mm",1,true);
    unavailable("autoHoleCompensation","Automatic hole compensation","Requires calibrated dimensional correction");
    number("elephantFoot","Elephant-foot compensation",0,2,"mm",1,true); check("preciseZ","Precise final layer height",true);
    section(0,"Ironing / wall generator"); unavailable("ironing","Ironing type","Ironing motion is not implemented","No ironing"); unavailable("wallGenerator","Wall generator","Variable-width Arachne is not integrated","Fixed width");
    section(0,"Advanced"); choice("outerWallFirst","Wall order",{"Inner / outer","Outer / inner"},true); check("infillFirst","Print infill first",true);
    for(const auto &item:QList<QPair<QString,QString>>{{"bridgeFlow","Bridge flow ratio"},{"thickBridges","Thick bridges"},{"counterbore","Counterbore bridges"},{"oneWallTop","Only one wall on top"},{"oneWallFirst","Only one wall on first layer"},{"smoothSpeedZone","Smooth speed change zone"},{"smoothCoefficient","Smoothing coefficient"},{"avoidCrossing","Avoid crossing walls"},{"zWallSmoothing","Z wall speed smoothing"}})
        unavailable(item.first,item.second,"Requires a qualified regional extrusion or routing algorithm");
    section(1,"Walls"); count("wallCount","Wall loops",0,50); check("alternateExtraWall","Alternate extra wall",true);
    unavailable("embedWalls","Embed wall in infill","Wall-infill overlap planning is not implemented"); unavailable("thinWalls","Detect thin walls","Adaptive thin-wall extrusion is not implemented");
    section(1,"Top / bottom shells");
    choice("topPattern","Top surface pattern",{"Rectilinear","Monotonic","Concentric"}); number("topDensity","Top surface density",0,100,"%",100); count("topShellLayers","Top shell layers",0,100); number("topThickness","Top shell thickness",0,20,"mm");
    unavailable("topColorLayers","Top color layers","One-extruder research preset");
    choice("bottomPattern","Bottom surface pattern",{"Rectilinear","Monotonic","Concentric"}); number("bottomDensity","Bottom surface density",0,100,"%",100); count("bottomShellLayers","Bottom shell layers",0,100); number("bottomThickness","Bottom shell thickness",0,20,"mm");
    unavailable("bottomColorLayers","Bottom color layers","One-extruder research preset"); unavailable("solidPattern","Internal solid pattern","Internal shells currently use monotonic strokes","Monotonic");
    section(1,"Infill"); number("infill","Sparse infill density",0,100,"%",100); choice("infillPattern","Sparse infill pattern",{"Stress aligned","Rectilinear","Grid"}); number("infillAngle","Infill direction",-360,360,"°",180/pi,true);
    section(1,"Strength / five-axis"); check("curved","Curved layers from stress"); check("profileNormal","Nozzle perpendicular to profile");
    bindings["profileNormal"].input->setToolTip("Slice along a centerline guide, without FEM. Process → Import profile guide, or Simulation → Open spiral five-axis test.");
    number("directionCone","Orientation search cone",0,89,"°",180/pi,true); number("maximumTilt","Maximum deposition tilt",0,89,"°",180/pi,true);
    section(1,"Standard XYZ · non-planar");check("nonPlanar3Axis","Conform layers to top surface");
    number("nonPlanarBase","Planar base height",0,100,"mm");number("nonPlanarSlope","Maximum surface slope",1,60,"°",180/pi);
    bindings["nonPlanar3Axis"].input->setToolTip("For standard XYZ printers: nozzle remains upright; layers above the planar base follow the model top.");
    section(2,"First layer speed"); number("firstLayerSpeed","First layer",0,1000,"mm/s"); number("firstLayerInfillSpeed","First layer infill",0,1000,"mm/s");
    section(2,"Other layer speeds"); number("speed","Default print",.1,1000,"mm/s");
    for(const auto &item:QList<QPair<QString,QString>>{{"outerWallSpeed","Outer wall"},{"innerWallSpeed","Inner wall"},{"infillSpeed","Sparse infill"},{"solidSpeed","Internal solid infill"},{"topSpeed","Top surface"},{"supportSpeed","Support"}}) number(item.first,item.second,0,1000,"mm/s");
    number("smallWallSpeed","Small wall speed",1,100,"%",100,true); number("smallWallThreshold","Small wall diameter threshold",0,50,"mm",1,true);
    number("smallWallAbsoluteSpeed","Small wall absolute speed (0 = %)",0,1000,"mm/s",1,true);
    unavailable("verticalShellSpeed","Vertical shell speed","Curved shell slope classification is not qualified");
    check("slowOverhangs","Slow down for overhangs",true);
    for(int i=0;i<5;++i) number(QString("overhang%1").arg(i),QString("Overhang speed · %1%").arg(i==0?10:i==1?25:i==2?50:i==3?75:100),0,1000,"mm/s",1,true);
    section(2,"Travel"); number("travelSpeed","Travel speed",.1,1000,"mm/s"); number("travelLift","Travel Z hop",0,30,"mm");
    section(3,"Support"); choice("supportMode","Generation",{"Normal","Reduced","No supports"});
    unavailable("supportType","Type","Tree supports are not implemented","Research columns"); unavailable("supportStyle","Style","Requires tree or conventional support routing","Columns");
    number("supportAngle","Threshold angle",0,89,"°",180/pi); check("supportBedOnly","Build plate only"); check("removeSmallOverhangs","Remove isolated overhangs",true);
    section(3,"Raft / filament"); unavailable("raftLayers","Raft layers","Rafts are not implemented","0"); unavailable("supportExtruder","Support / raft base filament","One extruder","Default"); unavailable("supportInterfaceExtruder","Support interface filament","One extruder","Default");
    section(3,"Advanced"); number("supportSpacing","Base pattern spacing (gap)",0,50,"mm"); number("supportReduction","Reduced spacing multiplier",1,20); number("maximumBridge","Maximum bridge span",0,50,"mm");
    for(const auto &item:QList<QPair<QString,QString>>{{"supportFirstDensity","First layer density"},{"supportFirstExpansion","First layer expansion"},{"supportWallLoops","Support wall loops"},{"supportTopGap","Top Z distance"},{"supportBottomGap","Bottom Z distance"}}) unavailable(item.first,item.second,"Research columns use one-layer contact spacing and validated deposition");
    section(4,"Bed adhesion"); count("skirtLoops","Skirt loops",0,50); count("skirtHeight","Skirt height (layers)",1,100); number("skirtDistance","Skirt distance",0,50,"mm",1,true);
    choice("brimType","Brim type",{"None","Automatic","Outer only","Inner only","Outer and inner"}); number("brimWidth","Brim width",0,50,"mm"); number("brimGap","Brim-object gap",0,5,"mm");
    section(4,"Prime tower");
    for(const auto &item:QList<QPair<QString,QString>>{{"purgeEnabled","Enable"},{"purgeSkip","Skip points"},{"purgeRib","Internal ribbing"},{"purgeWidth","Width"},{"purgeSpeed","Maximum speed"},{"purgeBrim","Brim width"},{"purgeGap","Infill gap"},{"purgeRibWall","Rib wall"},{"purgeExtraRib","Extra rib length"},{"purgeRibWidth","Rib width"},{"purgeFill","Fill wall"},{"wipeInfill","Wipe into infill"}})
        unavailable(item.first,item.second,"Multi-material purging is unavailable in the one-extruder research preset");
    for(auto column:columns) column->addStretch();
    connect(search,&QLineEdit::textChanged,this,[this]{filter();}); connect(advanced,&QCheckBox::toggled,this,[this]{filter();});
    set_settings(base);
}
void ProcessEditor::set_settings(const ProcessSettings &settings) {
    updating=true; base=settings; auto j=process_json(settings);
    for(auto it=bindings.begin();it!=bindings.end();++it) {
        auto key=it.key(); auto value=j[key];
        if(key=="nonPlanarBase")value=settings.nonPlanarBase;
        if(key=="nonPlanarSlope")value=settings.nonPlanarSlope;
        if(key.startsWith("overhang") && key.size()==9) value=settings.overhangSpeeds[key.right(1).toInt()];
        if(key=="supportMode") value=settings.supports?int(settings.supportMode):2;
        if(auto input=qobject_cast<QDoubleSpinBox*>(it->input)) input->setValue(value.toDouble()*it->scale);
        else if(auto input=qobject_cast<QSpinBox*>(it->input)) input->setValue(value.toInt());
        else if(auto input=qobject_cast<QCheckBox*>(it->input)) input->setChecked(value.toBool());
        else if(auto input=qobject_cast<QComboBox*>(it->input)) input->setCurrentIndex(value.isBool()?int(value.toBool()):value.toInt());
    }
    error.clear(); updating=false; filter();
}
void ProcessEditor::set_layout(Layout layout) {
    bool xyz=layout==Layout::Cartesian3Axis;
    auto group=bindings["nonPlanar3Axis"].row->parentWidget();group->setVisible(xyz);
    if(auto column=qobject_cast<QVBoxLayout*>(group->parentWidget()->layout()))column->insertWidget(xyz?0:column->count()-1,group);
    bindings["profileNormal"].row->parentWidget()->setVisible(!xyz);
    for(const auto &key:{"nonPlanar3Axis","nonPlanarBase","nonPlanarSlope"})bindings[key].input->setEnabled(xyz);
    for(const auto &key:{"profileNormal","curved","directionCone","maximumTilt"})bindings[key].input->setEnabled(!xyz);
}
ProcessSettings ProcessEditor::settings() const {
    auto j=process_json(base); auto overhang=j["overhangSpeeds"].toArray();
    for(auto it=bindings.begin();it!=bindings.end();++it) {
        QJsonValue value; auto key=it.key();
        if(auto input=qobject_cast<QDoubleSpinBox*>(it->input)) value=input->value()/it->scale;
        else if(auto input=qobject_cast<QSpinBox*>(it->input)) value=input->value();
        else if(auto input=qobject_cast<QCheckBox*>(it->input)) value=input->isChecked();
        else if(auto input=qobject_cast<QComboBox*>(it->input)) value=key=="outerWallFirst"?QJsonValue(bool(input->currentIndex())):QJsonValue(input->currentIndex());
        else continue;
        if(key.startsWith("overhang") && key.size()==9) overhang[key.right(1).toInt()]=value;
        else j[key]=value;
    }
    j["overhangSpeeds"]=overhang; j["supports"]=j["supportMode"].toInt()!=2;
    return read_process(j);
}
void ProcessEditor::changed() {
    if(updating) return;
    try { settings(); error.clear(); } catch(const std::exception &e) { error=e.what(); }
    emit edited();
}
void ProcessEditor::filter() {
    for(const auto &binding:bindings) binding.row->setVisible((advanced->isChecked() || !binding.advanced) &&
        (search->text().isEmpty() || binding.label.contains(search->text(),Qt::CaseInsensitive)));
}
