#include "main_window.h"
#include "core/geometry.h"
#include "core/process.h"
#include <QDialog>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>
using namespace fiveaxis;
void MainWindow::process_settings() {
    if(worker.isRunning()) return;
    QDialog dialog(this); dialog.setObjectName("processSettingsDialog");
    dialog.setWindowTitle("Print process · Orca options"); dialog.resize(560,700);
    auto outer=new QVBoxLayout(&dialog);
    auto note=new QLabel("Common Orca process controls adapted to curved layers. Zero widths, heights and print speeds inherit the default.");
    note->setWordWrap(true); outer->addWidget(note);
    auto tabs=new QTabWidget; outer->addWidget(tabs);
    auto tab=[&](const QString &name) { auto w=new QWidget; auto f=new QFormLayout(w); tabs->addTab(w,name); return f; };
    auto number=[&](QFormLayout *f,const QString &label,double value,double lo,double hi,const QString &name) {
        auto w=new QDoubleSpinBox; w->setRange(lo,hi); w->setDecimals(3); w->setValue(value); w->setObjectName(name); f->addRow(label,w); return w;
    };
    auto count=[&](QFormLayout *f,const QString &label,int value,int hi,const QString &name) {
        auto w=new QSpinBox; w->setRange(0,hi); w->setValue(value); w->setObjectName(name); f->addRow(label,w); return w;
    };
    auto combo=[&](QFormLayout *f,const QString &label,const QStringList &items,int value,const QString &name) {
        auto w=new QComboBox; w->addItems(items); w->setCurrentIndex(value); w->setObjectName(name); f->addRow(label,w); return w;
    };
    const auto &s=project.process;
    auto quality=tab("Quality");
    auto nozzle=number(quality,"Nozzle diameter · mm",s.nozzleDiameter,.1,3,"processNozzle");
    auto height=number(quality,"Layer height · mm",s.layerHeight,.01,2,"processLayerHeight");
    auto firstHeight=number(quality,"Initial layer height · mm",s.firstLayerHeight,0,2,"processFirstHeight");
    auto width=number(quality,"Default line width · mm",s.width,.01,5,"processWidth");
    auto firstWidth=number(quality,"Initial layer line width · mm",s.firstLayerWidth,0,5,"processFirstWidth");
    auto outerWidth=number(quality,"Outer wall width · mm",s.outerWallWidth,0,5,"processOuterWidth");
    auto innerWidth=number(quality,"Inner wall width · mm",s.innerWallWidth,0,5,"processInnerWidth");
    auto infillWidth=number(quality,"Sparse infill width · mm",s.infillWidth,0,5,"processInfillWidth");
    auto solidWidth=number(quality,"Solid infill width · mm",s.solidWidth,0,5,"processSolidWidth");
    auto seam=combo(quality,"Seam position",{"Lowest stress","Aligned","Back","Nearest","Random (deterministic)"},int(s.seam),"processSeam");
    auto defaults=new QPushButton("Use Orca default line width"); quality->addRow(defaults);
    connect(defaults,&QPushButton::clicked,&dialog,[=] { width->setValue(default_line_width(nozzle->value())); });
    auto strength=tab("Strength");
    auto walls=count(strength,"Wall loops",s.wallCount,50,"processWalls");
    auto order=combo(strength,"Wall order",{"Outer / inner","Inner / outer"},s.outerWallFirst?0:1,"processWallOrder");
    auto top=count(strength,"Top shell layers",s.topShellLayers,100,"processTopLayers");
    auto bottom=count(strength,"Bottom shell layers",s.bottomShellLayers,100,"processBottomLayers");
    auto density=number(strength,"Sparse infill density · %",s.infill*100,0,100,"processDensity");
    auto pattern=combo(strength,"Sparse infill pattern",{"Stress aligned","Rectilinear","Grid"},int(s.infillPattern),"processPattern");
    auto angle=number(strength,"Infill direction · degrees",s.infillAngle*180/pi,-360,360,"processInfillAngle");
    auto velocity=tab("Speed");
    auto base=number(velocity,"Default print speed · mm/s",s.speed,.1,1000,"processSpeed");
    auto outerSpeed=number(velocity,"Outer wall · mm/s",s.outerWallSpeed,0,1000,"processOuterSpeed");
    auto innerSpeed=number(velocity,"Inner wall · mm/s",s.innerWallSpeed,0,1000,"processInnerSpeed");
    auto sparseSpeed=number(velocity,"Sparse infill · mm/s",s.infillSpeed,0,1000,"processInfillSpeed");
    auto solidSpeed=number(velocity,"Solid infill · mm/s",s.solidSpeed,0,1000,"processSolidSpeed");
    auto supportSpeed=number(velocity,"Support · mm/s",s.supportSpeed,0,1000,"processSupportSpeed");
    auto firstSpeed=number(velocity,"Initial layer · mm/s",s.firstLayerSpeed,0,1000,"processFirstSpeed");
    auto travelSpeed=number(velocity,"Travel · mm/s",s.travelSpeed,.1,1000,"processTravelSpeed");
    auto lift=number(velocity,"Travel Z hop · mm",s.travelLift,0,30,"processTravelLift");
    auto support=tab("Support / five-axis");
    auto mode=combo(support,"Support generation",{"Normal","Reduced","No supports"},s.supports?int(s.supportMode):2,"processSupportMode");
    auto spacing=number(support,"Base pattern spacing (gap) · mm",s.supportSpacing,0,50,"processSupportSpacing");
    auto reduction=number(support,"Reduced spacing multiplier",s.supportReduction,1,20,"processSupportReduction");
    auto bridge=number(support,"Maximum bridge span · mm",s.maximumBridge,0,50,"processBridge");
    auto cone=number(support,"Orientation search cone · degrees",s.directionCone*180/pi,0,89,"processCone");
    auto tilt=number(support,"Maximum deposition tilt · degrees",s.maximumTilt*180/pi,0,89,"processTilt");
    auto info=new QLabel("Reduced mode uses fewer support columns. No supports generates zero support extrusion. Unsupported deposition remains highlighted and blocks export when it fails the configured process check.");
    info->setWordWrap(true); support->addRow(info);
    auto buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel); outer->addWidget(buttons);
    connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
    connect(buttons,&QDialogButtonBox::accepted,&dialog,[&] {
        try {
            ProcessSettings next=s;
            next.nozzleDiameter=nozzle->value(); next.layerHeight=height->value(); next.firstLayerHeight=firstHeight->value();
            next.width=width->value(); next.firstLayerWidth=firstWidth->value(); next.outerWallWidth=outerWidth->value();
            next.innerWallWidth=innerWidth->value(); next.infillWidth=infillWidth->value(); next.solidWidth=solidWidth->value();
            next.seam=SeamPosition(seam->currentIndex()); next.wallCount=walls->value(); next.outerWallFirst=order->currentIndex()==0;
            next.topShellLayers=top->value(); next.bottomShellLayers=bottom->value(); next.infill=density->value()/100;
            next.infillPattern=InfillPattern(pattern->currentIndex()); next.infillAngle=angle->value()*pi/180;
            next.speed=base->value(); next.outerWallSpeed=outerSpeed->value(); next.innerWallSpeed=innerSpeed->value();
            next.infillSpeed=sparseSpeed->value(); next.solidSpeed=solidSpeed->value(); next.supportSpeed=supportSpeed->value();
            next.firstLayerSpeed=firstSpeed->value(); next.travelSpeed=travelSpeed->value(); next.travelLift=lift->value();
            next.supportMode=SupportMode(mode->currentIndex()); next.supports=mode->currentIndex()!=2;
            next.supportSpacing=spacing->value(); next.supportReduction=reduction->value(); next.maximumBridge=bridge->value();
            next.directionCone=cone->value()*pi/180; next.maximumTilt=tilt->value()*pi/180;
            validate_process(next);
            if(next.nozzleDiameter!=project.process.nozzleDiameter) project.dummyToolhead=make_dummy_toolhead(next.nozzleDiameter);
            project.process=next; project.invalidate(); replaying=false;
            refresh(); log->append("Process settings updated. Slice again to apply them to the motion."); dialog.accept();
        } catch(const std::exception &e) { info->setText(e.what()); tabs->setCurrentIndex(3); }
    });
    dialog.exec();
}
void MainWindow::import_orca_settings() {
    if(worker.isRunning()) return;
    auto file=QFileDialog::getOpenFileName(this,"Import flattened Orca process preset",{},"Orca process JSON (*.json)");
    if(file.isEmpty()) return;
    try {
        QFile input(file); if(!input.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot open process preset");
        QJsonParseError error; auto doc=QJsonDocument::fromJson(input.readAll(),&error);
        if(error.error!=QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("Invalid process JSON");
        QStringList notes; auto next=import_orca_process(doc.object(),project.process,notes);
        if(next.nozzleDiameter!=project.process.nozzleDiameter) project.dummyToolhead=make_dummy_toolhead(next.nozzleDiameter);
        project.process=next; project.invalidate(); replaying=false; refresh();
        log->append("Imported supported Orca process settings. Slice again to apply them.");
        for(const auto &note:notes) log->append(note);
    } catch(const std::exception &e) { show_error(e.what()); }
}
