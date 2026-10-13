#include "app/main_window.h"
#include "core/geometry.h"
#include "core/project.h"
#include "app/step_dialog.h"
#include "core/step_import.h"
#include "core/motion_preview.h"
#include "core/profile.h"
#include "core/slicing.h"
#include "core/collision.h"
#include <QPushButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QSpinBox>
#include <QAction>
#include <QTabWidget>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace fiveaxis;
class DesktopTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
    void spiral_profile_preview_data(){QTest::addColumn<bool>("head");QTest::newRow("bed")<<false;QTest::newRow("head")<<true;}
    void spiral_profile_preview(){
        QFETCH(bool,head);auto p=spiral_profile_project(head?Layout::TiltingHead:Layout::RotaryBed);p.process.layerHeight=1.5;p.process.width=1.6;p.process.wallCount=1;p.process.infill=.2;try{slice_project(p);}catch(const std::exception &e){QFAIL(e.what());}
        QTemporaryDir dir;auto file=dir.filePath("spiral.5axis");save_project(p,file);MainWindow window;window.load(file);window.show();QVERIFY(QTest::qWaitForWindowExposed(&window));
        auto view=window.findChild<Viewport*>("simulationViewport");auto slider=window.findChild<QSlider*>("moveTimeline");auto checkbox=window.findChild<QCheckBox*>("profileNormal");QVERIFY(checkbox && checkbox->isChecked());
        size_t chosen=1;for(size_t i=1;i<p.path.size();++i)if(p.path[i].deltaE>0 && std::abs(p.path[i].joint.a)>.35){chosen=i;break;}
        double wanted=(p.path[chosen-1].time+p.path[chosen].time)/2;slider->setValue(int(wanted/p.path.back().time*10000));auto state=simulation_at(p,p.path.back().time*slider->value()/10000.0,false);
        QVERIFY(!view->axis_pose_active());QVERIFY((view->displayed_joint().xyz-state.joint.xyz).norm()<1e-9);
        auto sample=p.path[state.move];QVERIFY(sample.deltaE>0);QVERIFY((part_direction(p.machine,p.active_toolhead(),view->displayed_joint())-profile_layer(p,sample.layer).normal).norm()<1e-6);
        QVERIFY(window.findChild<QLabel*>("simulationHud")->text().contains("PROFILE NORMAL"));QVERIFY(!window.findChild<QPushButton*>("exportGcode")->isEnabled());
        slider->setValue(1500);slider->setValue(int(wanted/p.path.back().time*10000));QVERIFY((view->displayed_joint().xyz-state.joint.xyz).norm()<1e-9);window.close();
    }
    void rotary_axis_preview_data(){QTest::addColumn<bool>("head");QTest::newRow("rotary-bed")<<false;QTest::newRow("tilting-head")<<true;}
    void rotary_axis_preview(){
        QFETCH(bool,head);Project p;p.machine.layout=head?Layout::TiltingHead:Layout::RotaryBed;p.machine.minB=head?-75*pi/180:-pi;p.machine.maxB=-p.machine.minB;
        Component nozzle;nozzle.id=nozzle.name="Preview nozzle";nozzle.nozzle=true;nozzle.mesh=cylinder_mesh(Vec3(0,0,1),.3,2);p.toolhead.components.push_back(nozzle);
        p.part=box_mesh(Vec3(0,0,3),Vec3(12,8,6));
        for(auto angle:{std::make_pair(0.0,0.0),std::make_pair(25.0,35.0),std::make_pair(-10.0,-15.0)}){
            Sample s;s.joint=rotary_preview_pose(p.machine,Vec3(12,7,30),angle.first*pi/180,angle.second*pi/180);s.position=part_position(p.machine,s.joint);s.direction=part_direction(p.machine,p.active_toolhead(),s.joint);s.role=Role::Travel;p.path.push_back(s);
        }
        time_path(p);p.depositionValid=true;p.report=validate_motion(p);QVERIFY(p.report.exportable());
        QTemporaryDir dir;auto file=dir.filePath("axes.5axis");save_project(p,file);MainWindow window;window.load(file);window.show();QVERIFY(QTest::qWaitForWindowExposed(&window));
        auto view=window.findChild<Viewport*>("simulationViewport");auto source=window.findChild<QComboBox*>("axisPreviewSource");
        auto a=window.findChild<QDoubleSpinBox*>("previewAxisA"),b=window.findChild<QDoubleSpinBox*>("previewAxisB");auto slider=window.findChild<QSlider*>("moveTimeline");auto exportButton=window.findChild<QPushButton*>("exportGcode");
        QVERIFY(view && source && a && b && exportButton);QVERIFY(exportButton->isEnabled());
        slider->setValue(5000);auto planned=simulation_at(p,p.path.back().time*.5,false).joint;QVERIFY(!view->axis_pose_active());QVERIFY(std::abs(view->displayed_joint().a-planned.a)<1e-9);
        auto fingerprint=window.current_project().fingerprint();auto before=view->grabFramebuffer();Vec3 anchor=part_position(p.machine,planned);
        a->setValue(30);b->setValue(40);QVERIFY(view->axis_pose_active());QCOMPARE(source->currentIndex(),1);QVERIFY(!exportButton->isEnabled());
        auto pose=view->displayed_joint();QVERIFY(std::abs(pose.a-pi/6)<1e-9);QVERIFY(std::abs(pose.b-40*pi/180)<1e-9);QVERIFY((part_position(p.machine,pose)-anchor).norm()<1e-9);
        view->repaint();QVERIFY(view->grabFramebuffer()!=before);QCOMPARE(window.current_project().fingerprint(),fingerprint);
        QVERIFY(window.findChild<QTextEdit*>("moveCodePreview")->toPlainText().contains("No generated G-code"));
        source->setCurrentIndex(2);QTest::qWait(120);auto animated=view->displayed_joint();QTest::qWait(140);
        QVERIFY(std::abs(animated.a-view->displayed_joint().a)>1e-4);QVERIFY(std::abs(animated.b-view->displayed_joint().b)>1e-4);QCOMPARE(window.current_project().fingerprint(),fingerprint);
        slider->setValue(3000);QVERIFY(!view->axis_pose_active());QCOMPARE(source->currentIndex(),0);QVERIFY(exportButton->isEnabled());
        auto expected=simulation_at(p,p.path.back().time*.3,false).joint;QVERIFY((view->displayed_joint().xyz-expected.xyz).norm()<1e-9);
        a->setValue(30);b->setValue(40);auto shots=qEnvironmentVariable("FIVEAXIS_AXIS_SCREENSHOTS");
        window.capture(dir.filePath("manual.png"));QVERIFY(view->axis_pose_active());QVERIFY(std::abs(view->displayed_joint().a-pi/6)<1e-9);
        if(!shots.isEmpty()){QApplication::processEvents();window.grab().save(shots+(head?"/axes-head-manual.png":"/axes-bed-manual.png"));}
        window.findChild<QPushButton*>("resetPreviewAxes")->click();QVERIFY(std::abs(view->displayed_joint().a)<1e-9);QVERIFY(std::abs(view->displayed_joint().b)<1e-9);
        source->setCurrentIndex(0);QVERIFY(!view->axis_pose_active());QCOMPARE(window.current_project().fingerprint(),fingerprint);
        p.path.resize(1);auto carriage=controller_axes(p.machine,p.toolhead,p.path.front().joint).xyz;
        p.machine.minXYZ=carriage-Vec3::Constant(.01);p.machine.maxXYZ=carriage+Vec3::Constant(.01);p.invalidate();save_project(p,file);window.load(file);
        a->setValue(30);QVERIFY(view->axis_pose_active());QVERIFY(std::abs(view->displayed_joint().a)<1e-9);
        QVERIFY(window.findChild<QLabel*>("axisPreviewStatus")->text().contains("Carriage limit reached"));window.close();
    }
    void inline_workspace_and_layer_filters() {
        MainWindow window; QTemporaryDir dir; auto p=demo_project(); p.path.clear(); p.part=box_mesh(Vec3(0,0,1),Vec3(6,6,2));
        Sample start; start.position=start.joint.xyz=Vec3(-2,-2,.2); p.path.push_back(start);
        for(int layer=0;layer<3;++layer) for(int move=0;move<4;++move) {
            auto s=p.path.back(); s.position=s.joint.xyz=Vec3(move%2?2:-2,move<2?-2:2,.2+layer*.2); s.layer=layer;
            s.role=move%2?Role::OuterWall:Role::Infill; s.deltaE=.1; s.time+=1; p.path.push_back(s);
        }
        save_project(p,dir.filePath("workspace.5axis")); window.load(dir.filePath("workspace.5axis")); window.show(); QVERIFY(QTest::qWaitForWindowExposed(&window));
        auto editor=window.findChild<ProcessEditor*>("processEditor"); QVERIFY(editor); QCOMPARE(editor->tabs()->count(),5);
        auto preset=window.findChild<QComboBox*>("processPreset"); preset->setCurrentIndex(1);
        QCOMPARE(window.current_project().process.layerHeight,.3); QCOMPARE(window.current_project().process.nozzleDiameter,.6); QVERIFY(!window.current_project().process.supports);
        auto width=editor->findChild<QDoubleSpinBox*>("width"); auto slice=window.findChild<QPushButton*>("slicePlate");
        width->setValue(.1); QVERIFY(!editor->valid()); QVERIFY(!slice->isEnabled());
        width->setValue(.6); QVERIFY(editor->valid()); QVERIFY(slice->isEnabled()); QCOMPARE(window.current_project().process.width,.6);
        editor->findChild<QSpinBox*>("wallCount")->setValue(3); QCOMPARE(window.current_project().process.wallCount,3);
        editor->findChild<QCheckBox*>("infillFirst")->setChecked(true); QVERIFY(window.current_project().process.infillFirst);
        editor->findChild<QComboBox*>("brimType")->setCurrentIndex(2); QCOMPARE(window.current_project().process.brimType,2);
        QVERIFY(!editor->findChild<QLineEdit*>("scarfSmart")->isEnabled()); QVERIFY(!editor->findChild<QLineEdit*>("purgeEnabled")->isEnabled());
        auto view=window.findChild<Viewport*>("simulationViewport"); auto high=window.findChild<QSlider*>("upperLayer"); auto low=window.findChild<QSlider*>("lowerLayer");
        QCOMPARE(high->maximum(),2); high->setValue(1); QCOMPARE(view->last_layer(),1); low->setValue(2); QCOMPARE(high->value(),2); QCOMPARE(view->first_layer(),2);
        low->setValue(0); high->setValue(0); QCOMPARE(view->last_layer(),0);
        high->setValue(2); auto roles=window.findChild<QTableWidget*>("roleStatistics"); auto before=view->grabFramebuffer();
        roles->item(int(Role::OuterWall),0)->setCheckState(Qt::Unchecked); view->repaint(); QVERIFY(before!=view->grabFramebuffer());
        high->setValue(1); auto cost=window.findChild<QDoubleSpinBox*>("filamentCost"); cost->setValue(50); QCOMPARE(window.current_project().filamentCost,50.0); QCOMPARE(view->last_layer(),1);high->setValue(2);
        QVERIFY(window.findChild<QTextEdit*>("moveCodePreview")->toPlainText().contains("PLANNED MOTION"));
        auto shots=qEnvironmentVariable("FIVEAXIS_SCREENSHOTS"); if(!shots.isEmpty()) {
            const QStringList names={"quality","strength","speed","support","other"};
            for(int i=0;i<5;++i) {editor->tabs()->setCurrentIndex(i); QApplication::processEvents(); window.grab().save(shots+"/workspace-"+names[i]+".png");}
            window.findChild<QTabBar*>("workspaceTabs")->setCurrentIndex(0); QApplication::processEvents(); window.grab().save(shots+"/workspace-prepare.png");
            window.resize(1280,800); QApplication::processEvents(); window.grab().save(shots+"/workspace-1280.png");
        }
        window.close();
    }
    void step_parameters_preview() {
        QTemporaryDir dir; auto path=dir.filePath("preview.step"); write_demo_step(path,true); MainWindow parent;
        ToolheadAssembly assembly; bool loaded=false; QTimer poll; poll.setInterval(50);
        connect(&poll,&QTimer::timeout,&parent,[&] {
            auto dialog=parent.findChild<QDialog*>("stepImportDialog"); if(!dialog) return;
            auto buttons=dialog->findChild<QDialogButtonBox*>(); if(!buttons->button(QDialogButtonBox::Ok)->isEnabled()) return;
            auto label=dialog->findChild<QLabel*>("stepFacetCount"); loaded=label && label->text().contains("collision facets");
            auto shots=qEnvironmentVariable("FIVEAXIS_SCREENSHOTS"); if(!shots.isEmpty()) dialog->grab().save(shots+"/step-import-parameters.png");
            buttons->button(QDialogButtonBox::Ok)->click();
        });
        poll.start(); QTimer::singleShot(15000,&parent,[&]{if(auto dialog=parent.findChild<QDialog*>("stepImportDialog")) dialog->reject();});
        QVERIFY(import_step_dialog(&parent,path,assembly,true)); QVERIFY(loaded); QVERIFY(assembly.components.size()>=4);
    }
    void toolpaths_dummy_and_process_controls() {
        QSurfaceFormat format; format.setDepthBufferSize(24); format.setVersion(2,1);
        format.setProfile(QSurfaceFormat::CompatibilityProfile); QSurfaceFormat::setDefaultFormat(format);
        MainWindow window; QTemporaryDir dir;
        auto p=Project{}; Component c; c.id=c.name="Stored assembly"; c.mesh=box_mesh(Vec3(0,0,2),Vec3(.2,.2,.2));
        p.toolhead.components.push_back(c);
        Sample a,b; a.position=a.joint.xyz=Vec3(-15,0,30); b=a; b.position=b.joint.xyz=Vec3(15,0,30); b.time=1;
        p.path={a,b}; p.depositionValid=true;
        save_project(p,dir.filePath("controls.5axis")); window.load(dir.filePath("controls.5axis")); window.show();
        QVERIFY(QTest::qWaitForWindowExposed(&window)); QTest::qWait(150);
        auto view=window.findChild<Viewport*>("simulationViewport"); auto paths=window.findChild<QComboBox*>("toolpathMode");
        auto support=window.findChild<QComboBox*>("supportMode"); auto dummy=window.findChild<QCheckBox*>("dummyToolhead");
        QVERIFY(view && paths && support && dummy); QVERIFY(view->toolpath_segment_count()>0);
        auto visible=view->grabFramebuffer();
        if(!qEnvironmentVariable("FIVEAXIS_DEBUG_FRAME").isEmpty()) visible.save(qEnvironmentVariable("FIVEAXIS_DEBUG_FRAME"));
        auto dimBluePixels=[](const QImage &image) {
            int count=0;
            for(int y=0;y<image.height();++y) for(int x=0;x<image.width();++x) {
                auto c=image.pixelColor(x,y);
                if(std::abs(c.red()-193)<=6 && std::abs(c.green()-212)<=6 && std::abs(c.blue()-237)<=6) ++count;
            }
            return count;
        };
        QVERIFY(dimBluePixels(visible)>20);
        view->repaint(); QVERIFY(dimBluePixels(view->grabFramebuffer())>20);
        paths->setCurrentIndex(3); QTest::qWait(50);
        QCOMPARE(view->path_mode(),3); auto hidden=view->grabFramebuffer(); QVERIFY(visible!=hidden);
        paths->setCurrentIndex(2); QCOMPARE(view->path_mode(),2); paths->setCurrentIndex(0);
        support->setCurrentIndex(1); QCOMPARE(window.current_project().process.supportMode,SupportMode::Reduced);
        support->setCurrentIndex(2); QVERIFY(!window.current_project().process.supports);
        support->setCurrentIndex(0);
        dummy->setChecked(true); QTRY_VERIFY_WITH_TIMEOUT(window.current_project().report.exportable(),15000);
        QVERIFY(window.current_project().useDummyToolhead); QCOMPARE(window.current_project().toolhead.components[0].name,QString("Stored assembly"));
        dummy->setChecked(false); QTRY_VERIFY_WITH_TIMEOUT(window.current_project().report.exportable(),15000);
        QVERIFY(!window.current_project().useDummyToolhead);
        QAction *settings=nullptr;
        for(auto action:window.findChildren<QAction*>()) if(action->text()=="Process settings · Orca options") settings=action;
        QVERIFY(settings);
        bool dialogVisited=false;
        QTimer::singleShot(50,&window,[&] {
            auto dialog=window.findChild<QDialog*>("processSettingsDialog"); if(!dialog) return;
            dialogVisited=true;
            auto walls=dialog->findChild<QSpinBox*>("wallCount"); walls->setValue(3);
            dialog->findChild<QSpinBox*>("topShellLayers")->setValue(2);
            dialog->findChild<QComboBox*>("infillPattern")->setCurrentIndex(2);
            dialog->findChild<QComboBox*>("supportMode")->setCurrentIndex(1);
            auto screenshots=qEnvironmentVariable("FIVEAXIS_SCREENSHOTS");
            if(!screenshots.isEmpty()) {
                auto tabs=dialog->findChild<QTabWidget*>(); tabs->setCurrentIndex(3); QApplication::processEvents();
                dialog->grab().save(screenshots+"/process-support-settings.png");
                tabs->setCurrentIndex(0); QApplication::processEvents();
                dialog->grab().save(screenshots+"/process-quality-settings.png");
            }
            auto buttons=dialog->findChild<QDialogButtonBox*>(); buttons->button(QDialogButtonBox::Ok)->click();
        });
        settings->trigger(); QVERIFY(dialogVisited);
        QCOMPARE(window.current_project().process.wallCount,3); QCOMPARE(window.current_project().process.topShellLayers,2);
        QCOMPARE(window.current_project().process.infillPattern,InfillPattern::Grid);
        QCOMPARE(window.current_project().process.supportMode,SupportMode::Reduced);
        QVERIFY(!window.current_project().report.exportable());
        view->grabFramebuffer().save(dir.filePath("toolpaths.png")); window.close();
    }
    void playback_and_background_replay() {
        QSurfaceFormat format;
        format.setDepthBufferSize(24);
        format.setVersion(2, 1);
        format.setProfile(QSurfaceFormat::CompatibilityProfile);
        QSurfaceFormat::setDefaultFormat(format);
        MainWindow window;
        QTemporaryDir dir;
        Project p;
        Component c;
        c.id = c.name = "UI probe";
        c.mesh = box_mesh(Vec3(0, 0, 1), Vec3(.2, .2, .2));
        p.toolhead.components.push_back(c);
        Sample a, b;
        a.position = a.joint.xyz = Vec3(0, 0, 5);
        b = a;
        b.position = b.joint.xyz = Vec3(2, 0, 5);
        b.time = 1;
        p.path = {a, b};
        p.depositionValid = true;
        auto file = dir.filePath("ui.5axis");
        save_project(p, file);
        window.load(file);
        window.show();
        QVERIFY(QTest::qWaitForWindowExposed(&window));
        QTest::qWait(100);
        auto timeline = window.findChild<QSlider *>("moveTimeline"); QVERIFY(timeline);
        timeline->setValue(7500);
        QCOMPARE(timeline->value(), 7500);
        timeline->setValue(1000);
        QCOMPARE(timeline->value(), 1000);
        QPushButton *play = nullptr;
        QComboBox *controller = nullptr;
        QComboBox *mode = nullptr;
        for (auto button : window.findChildren<QPushButton *>())
            if (button->text() == "Play / pause")
                play = button;
        for (auto combo : window.findChildren<QComboBox *>()) {
            if (combo->itemText(0) == "Duet / RepRapFirmware")
                controller = combo;
            if (combo->itemText(0) == "Planned motion")
                mode = combo;
        }
        QVERIFY(play && controller && mode);
        QTest::mouseClick(play, Qt::LeftButton);
        QTest::qWait(180);
        QVERIFY(timeline->value() > 1000);
        QTest::mouseClick(play, Qt::LeftButton);
        controller->setCurrentIndex(1);
        auto ready = [&] {
            for (auto label : window.findChildren<QLabel *>())
                if (label->text().startsWith("Motion validated"))
                    return true;
            return false;
        };
        QTRY_VERIFY_WITH_TIMEOUT(ready(), 15000);
        mode->setCurrentIndex(1);
        auto replayReady = [&] {
            for (auto label : window.findChildren<QLabel *>())
                if (label->text().contains("Geometric motion checked"))
                    return true;
            return false;
        };
        QTRY_VERIFY_WITH_TIMEOUT(replayReady(), 15000);
        timeline->setValue(9000);
        timeline->setValue(1000);
        QCOMPARE(timeline->value(), 1000);
        window.close();
    }
};
QTEST_MAIN(DesktopTests)
#include "desktop_tests.moc"
