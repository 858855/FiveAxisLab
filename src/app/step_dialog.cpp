#include "step_dialog.h"
#include "viewport.h"
#include <QDialog>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QSignalBlocker>
#include <QTimer>
#include <QHeaderView>
#include <QtConcurrent/QtConcurrentRun>
using namespace fiveaxis;
bool import_step_dialog(QWidget *parent,const QString &file,ToolheadAssembly &assembly,bool toolhead) {
    struct Result { ToolheadAssembly assembly; QString error; };
    Project preview; QDialog dialog(parent); dialog.setObjectName("stepImportDialog"); dialog.setWindowTitle("STEP file import parameters"); dialog.resize(1050,650);
    if(parent) dialog.setStyleSheet(parent->styleSheet());
    auto horizontal=new QHBoxLayout(&dialog); auto panel=new QVBoxLayout; horizontal->addLayout(panel);
    auto note=new QLabel("Smaller linear and angular deflections produce finer meshes. Collision clearance includes the linear tessellation allowance."); note->setWordWrap(true); note->setMaximumWidth(370); panel->addWidget(note);
    auto form=new QFormLayout; panel->addLayout(form);
    auto tolerance=[&](const QString &name,const QString &label,double initial,double low,double high,const QString &unit) {
        auto container=new QWidget; auto row=new QHBoxLayout(container); row->setContentsMargins(0,0,0,0);
        auto slider=new QSlider(Qt::Horizontal); slider->setRange(1,1000);
        auto spin=new QDoubleSpinBox; spin->setObjectName(name); spin->setDecimals(4); spin->setRange(low,high); spin->setValue(initial); spin->setSuffix(" "+unit); spin->setKeyboardTracking(false); spin->setFixedWidth(112);
        slider->setValue(int((initial-low)/(high-low)*999)+1); row->addWidget(slider); row->addWidget(spin); form->addRow(label,container);
        QObject::connect(slider,&QSlider::valueChanged,spin,[=](int value){ spin->setValue(low+(high-low)*(value-1)/999); });
        QObject::connect(spin,qOverload<double>(&QDoubleSpinBox::valueChanged),slider,[=](double value){ QSignalBlocker guard(slider); slider->setValue(int((value-low)/(high-low)*999)+1); });
        return spin;
    };
    auto linear=tolerance("stepLinearDeflection","Linear deflection",.02,.001,.5,"mm");
    auto angular=tolerance("stepAngularDeflection","Angular deflection",.0872664626,.01,1,"rad");
    auto render=tolerance("stepRenderDeflection","Rendering deflection",.1,.01,1,"mm");
    auto split=new QCheckBox("Split Compound / CompSolid into solids"); split->setObjectName("stepSplitCompounds"); panel->addWidget(split);
    auto facets=new QLabel("Reading STEP…"); facets->setWordWrap(true); facets->setMinimumHeight(56); facets->setObjectName("stepFacetCount"); panel->addWidget(facets);
    auto tree=new QTreeWidget; tree->setHeaderLabels({"Component","Collision facets"}); tree->setMinimumWidth(360); tree->header()->setSectionResizeMode(QHeaderView::ResizeToContents); panel->addWidget(tree,1);
    auto view=new Viewport; view->setMinimumSize(500,400); horizontal->addWidget(view,1);
    view->set_rotary_axes_visible(false);
    auto buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel); panel->addWidget(buttons); buttons->button(QDialogButtonBox::Ok)->setEnabled(false);
    QFutureWatcher<Result> worker; auto token=std::make_shared<std::atomic_bool>(false); bool dirty=false;
    std::function<void()> build=[&] {
        if(worker.isRunning()) { dirty=true; token->store(true); return; }
        dirty=false; token=std::make_shared<std::atomic_bool>(false); auto cancelled=token;
        StepImportOptions options{linear->value(),angular->value(),render->value(),split->isChecked()};
        buttons->button(QDialogButtonBox::Ok)->setEnabled(false); facets->setText("Tessellating in background…");
        worker.setFuture(QtConcurrent::run([file,options,cancelled] { Result r; try { r.assembly=load_step_assembly(file,options,cancelled.get()); } catch(const std::exception &e){r.error=e.what();} return r; }));
    };
    QObject::connect(&worker,&QFutureWatcher<Result>::finished,&dialog,[&] {
        if(dirty) { build(); return; }
        auto result=worker.result(); if(!result.error.isEmpty()) { facets->setText(result.error); return; }
        assembly=std::move(result.assembly); preview.toolhead=assembly; tree->clear(); size_t count=0;
        for(const auto &component:assembly.components) { auto item=new QTreeWidgetItem(tree,{component.name,QString::number(component.mesh.triangles.size())}); item->setToolTip(0,component.id); count+=component.mesh.triangles.size(); }
        facets->setText(QString("%1 collision facets · %2 components\nSource units: %3 → mm").arg(count).arg(assembly.components.size()).arg(assembly.sourceUnits.trimmed()));
        view->set_project(&preview); view->fit(); buttons->button(QDialogButtonBox::Ok)->setEnabled(true);
    });
    QTimer debounce; debounce.setSingleShot(true); debounce.setInterval(300); QObject::connect(&debounce,&QTimer::timeout,&dialog,build);
    for(auto input:{linear,angular,render}) QObject::connect(input,qOverload<double>(&QDoubleSpinBox::valueChanged),&dialog,[&]{buttons->button(QDialogButtonBox::Ok)->setEnabled(false);debounce.start();});
    QObject::connect(split,&QCheckBox::toggled,&dialog,[&]{buttons->button(QDialogButtonBox::Ok)->setEnabled(false);debounce.start();});
    QObject::connect(buttons,&QDialogButtonBox::accepted,&dialog,[&]{if(!debounce.isActive() && !worker.isRunning()) dialog.accept();});
    QObject::connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
    if(!toolhead) note->setText(note->text()+" Split solids retain their names in this tree; generation currently treats the imported part as one global object.");
    build(); bool accepted=dialog.exec()==QDialog::Accepted; token->store(true); debounce.stop(); worker.waitForFinished(); return accepted;
}
