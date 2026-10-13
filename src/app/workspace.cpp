#include "main_window.h"
#include "core/controllers.h"
#include <QHeaderView>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QSignalBlocker>
#include <QTextCursor>
#include <QTextCharFormat>
#include <array>
using namespace fiveaxis;
static QString duration_text(double seconds) {
    int n=int(std::round(seconds)); return n>=3600?QString("%1h %2m").arg(n/3600).arg(n%3600/60):QString("%1m %2s").arg(n/60).arg(n%60);
}
static QString short_time(double seconds) {int n=int(std::round(seconds)); return n>=3600?QString::number(n/3600.0,'f',1)+"h":n>=60?QString::number(n/60.0,'f',1)+"m":QString::number(n)+"s";}
QWidget *MainWindow::create_preview_inspector() {
    auto panel=new QWidget; panel->setObjectName("previewInspector"); panel->setFixedWidth(300);
    auto outer=new QVBoxLayout(panel); outer->setContentsMargins(4,4,4,4); outer->setSpacing(5);
    auto heading=new QLabel("Slicing result · line type"); heading->setStyleSheet("font-weight:600;"); outer->addWidget(heading);
    roleStatistics=new QTableWidget(role_count,5); roleStatistics->setObjectName("roleStatistics");
    roleStatistics->setHorizontalHeaderLabels({"Line type","Time","%","m","g"}); roleStatistics->verticalHeader()->hide(); roleStatistics->setShowGrid(false);
    roleStatistics->setEditTriggers(QAbstractItemView::NoEditTriggers); roleStatistics->setSelectionMode(QAbstractItemView::NoSelection);
    roleStatistics->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); roleStatistics->horizontalHeader()->setSectionResizeMode(0,QHeaderView::Fixed); roleStatistics->setColumnWidth(0,112);
    roleStatistics->horizontalHeader()->setSectionResizeMode(1,QHeaderView::Fixed);roleStatistics->setColumnWidth(1,50);
    roleStatistics->setMinimumHeight(255); roleStatistics->setMaximumHeight(340); outer->addWidget(roleStatistics);
    for(int r=0;r<role_count;++r) {
        auto item=new QTableWidgetItem(role_name(Role(r))); item->setFlags(Qt::ItemIsEnabled|Qt::ItemIsUserCheckable); item->setCheckState(Qt::Checked); item->setForeground(toolpath_color(Role(r)));
        roleStatistics->setItem(r,0,item); roleStatistics->setRowHeight(r,23);
        for(int col=1;col<5;++col) roleStatistics->setItem(r,col,new QTableWidgetItem("0"));
    }
    connect(roleStatistics,&QTableWidget::itemChanged,this,[this](QTableWidgetItem *item) {
        if(!updatingPreview && item->column()==0) viewport->set_role_visible(Role(item->row()),item->checkState()==Qt::Checked);
    });
    totals=new QLabel; totals->setObjectName("filamentEstimates"); totals->setWordWrap(true); outer->addWidget(totals);
    auto range=new QGroupBox("Layer range"); auto rangeRow=new QHBoxLayout(range); rangeRow->setContentsMargins(5,12,5,5);
    lowerLayer=new QSlider(Qt::Vertical); lowerLayer->setObjectName("lowerLayer"); upperLayer=new QSlider(Qt::Vertical); upperLayer->setObjectName("upperLayer");
    lowerLayer->setMinimumHeight(80); upperLayer->setMinimumHeight(80); layerLabel=new QLabel; layerLabel->setWordWrap(true); rangeRow->addWidget(layerLabel,1); rangeRow->addWidget(lowerLayer); rangeRow->addWidget(upperLayer); outer->addWidget(range);
    auto updateRange=[this](bool upper) {
        if(updatingPreview) return;
        QSignalBlocker low(lowerLayer),high(upperLayer);
        if(lowerLayer->value()>upperLayer->value()) { if(upper) lowerLayer->setValue(upperLayer->value()); else upperLayer->setValue(lowerLayer->value()); }
        int lo=lowerLayer->value(),hi=upperLayer->value(); viewport->set_layer_range(lo,hi);
        layerLabel->setText(QString("Layers %1 – %2\n(%3 total)\nGeometry filters only").arg(lo+1).arg(hi+1).arg(upperLayer->maximum()+1));
        auto &p=replaying?replay:project; double end=0;
        for(const auto &move:p.path) if(move.layer==hi) end=std::max(end,move.time);
        playing=false; seek(end);
    };
    connect(lowerLayer,&QSlider::valueChanged,this,[updateRange]{updateRange(false);}); connect(upperLayer,&QSlider::valueChanged,this,[updateRange]{updateRange(true);});
    auto label=new QLabel("Current movement"); label->setStyleSheet("font-weight:600;"); outer->addWidget(label);
    movePreview=new QTextEdit; movePreview->setObjectName("moveCodePreview"); movePreview->setReadOnly(true); movePreview->setMinimumHeight(150);
    movePreview->setStyleSheet("font-family:Consolas;font-size:10px;background:#394345;color:#e9eded;"); outer->addWidget(movePreview,1);
    return panel;
}
void MainWindow::refresh_preview(bool preserveRange) {
    updatingPreview=true; auto &p=replaying?replay:project;
    std::array<double,role_count> times{},extrusion{}; int maximum=0;
    for(size_t i=0;i<p.path.size();++i) {
        const auto &s=p.path[i]; int role=int(s.role); maximum=std::max(maximum,s.layer);
        if(role>=0 && role<role_count) { times[role]+=i?std::max(0.0,s.time-p.path[i-1].time):0; extrusion[role]+=std::max(0.0,s.deltaE); }
    }
    double total=p.path.empty()?0:p.path.back().time,filament=0,area=pi*std::pow(p.machine.filamentDiameter/2,2);
    for(int r=0;r<role_count;++r) {
        filament+=extrusion[r]; auto write=[&](int c,const QString &text){roleStatistics->item(r,c)->setText(text);};
        write(1,short_time(times[r])); write(2,QString::number(total>0?times[r]/total*100:0,'f',1));
        write(3,QString::number(extrusion[r]/1000,'f',2)); write(4,QString::number(extrusion[r]*area/1000*project.filamentDensity,'f',2));
    }
    double grams=filament*area/1000*project.filamentDensity;
    totals->setText(QString("Total filament: %1 m · %2 g\nMaterial cost: %3 EUR\nPlanned time: %4\n%5\n%6")
        .arg(filament/1000,0,'f',2).arg(grams,0,'f',2).arg(grams/1000*project.filamentCost,0,'f',2).arg(duration_text(total)).arg(project.filamentName)
        .arg(project.useDummyToolhead?"Dummy toolhead · synthetic geometry":"Configured toolhead geometry"));
    int lo=preserveRange?std::min(lowerLayer->value(),maximum):0,hi=preserveRange?std::min(upperLayer->value(),maximum):maximum;
    lowerLayer->setRange(0,maximum); upperLayer->setRange(0,maximum); lowerLayer->setValue(lo); upperLayer->setValue(hi);
    viewport->set_layer_range(lo,hi); layerLabel->setText(p.path.empty()?"No sliced layers":QString("Layers %1 – %2\nGeometry filters only").arg(lo+1).arg(hi+1));
    lastPreviewMove=-1; updatingPreview=false; update_move_preview();
}
void MainWindow::update_move_preview() {
    auto &p=replaying?replay:project;
    if(p.path.empty()) { movePreview->setPlainText("Slice a part to inspect motion."); return; }
    auto state=simulation_at(p,time,false); if(state.move==lastPreviewMove) return; lastPreviewMove=state.move;
    movePreview->clear();
    movePreview->append(replaying?"CONTROLLER REPLAY · relative E":"PLANNED MOTION · relative E\nCoordinates below are a preview; export requires validation.");
    for(int i=std::max(0,state.move-3);i<std::min<int>(p.path.size(),state.move+8);++i) {
        const auto &s=p.path[i]; auto axes=controller_axes(p.machine,p.active_toolhead(),s.joint);
        QTextCharFormat format; format.setForeground(i==state.move?QColor("#00eb76"):QColor("#d5dddd")); format.setFontWeight(i==state.move?QFont::Bold:QFont::Normal);
        format.setFontPointSize(9);
        double feed=0; if(i>0) {const auto &a=p.path[i-1];auto previous=controller_axes(p.machine,p.active_toolhead(),a.joint);feed=feed_metric(p.machine,previous,axes,s.deltaE)/std::max(.001,s.time-a.time)*60;}
        auto cursor=movePreview->textCursor(); cursor.movePosition(QTextCursor::End); cursor.insertText(QString("\n%1 · L%2 · %3\n%13 X%4 Y%5 Z%6 A%7 %8%9 E%10 F%14\n; t=%11 s · requested %12 mm/s\n")
            .arg(i).arg(s.layer+1).arg(role_name(s.role)).arg(axes.xyz.x(),0,'f',3).arg(axes.xyz.y(),0,'f',3).arg(axes.xyz.z(),0,'f',3)
            .arg(s.joint.a*180/pi,0,'f',3).arg(p.machine.axis2()).arg(s.joint.b*180/pi,0,'f',3).arg(s.deltaE,0,'f',5).arg(s.time,0,'f',2).arg(s.speed,0,'f',1).arg(i==0?"Initial pose":"G1").arg(feed,0,'f',2),format);
    }
    movePreview->moveCursor(QTextCursor::Start);
}
void MainWindow::set_workspace(int index) {
    QSignalBlocker guard(workspaces); workspaces->setCurrentIndex(index); viewport->set_prepare(index==0);
    auto inspector=findChild<QWidget*>("previewInspector"); if(inspector) inspector->setVisible(index==1);
    for(const auto &name:{"simulationControls","simulationPaths","simulationTransport"}) if(auto widget=findChild<QWidget*>(name)) widget->setVisible(index==1);
    if(index==1) viewport->set_time(time);
}
