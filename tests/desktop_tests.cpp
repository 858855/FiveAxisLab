#include "app/main_window.h"
#include "core/geometry.h"
#include "core/project.h"
#include <QPushButton>
#include <QDialog>
#include <QDialogButtonBox>
#include <QSpinBox>
#include <QTabWidget>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace fiveaxis;
class DesktopTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
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
        auto visible=view->grabFramebuffer(); paths->setCurrentIndex(3); QTest::qWait(50);
        QCOMPARE(view->path_mode(),3); auto hidden=view->grabFramebuffer(); QVERIFY(visible!=hidden);
        paths->setCurrentIndex(2); QCOMPARE(view->path_mode(),2); paths->setCurrentIndex(0);
        support->setCurrentIndex(1); QCOMPARE(window.current_project().process.supportMode,SupportMode::Reduced);
        support->setCurrentIndex(2); QVERIFY(!window.current_project().process.supports);
        support->setCurrentIndex(0);
        dummy->setChecked(true); QTRY_VERIFY_WITH_TIMEOUT(window.current_project().report.exportable(),15000);
        QVERIFY(window.current_project().useDummyToolhead); QCOMPARE(window.current_project().toolhead.components[0].name,QString("Stored assembly"));
        dummy->setChecked(false); QTRY_VERIFY_WITH_TIMEOUT(window.current_project().report.exportable(),15000);
        QVERIFY(!window.current_project().useDummyToolhead);
        QPushButton *settings=nullptr;
        for(auto b:window.findChildren<QPushButton*>()) if(b->text()=="Process settings · Orca options") settings=b;
        QVERIFY(settings);
        bool dialogVisited=false;
        QTimer::singleShot(50,&window,[&] {
            auto dialog=window.findChild<QDialog*>("processSettingsDialog"); if(!dialog) return;
            dialogVisited=true;
            auto walls=dialog->findChild<QSpinBox*>("processWalls"); walls->setValue(3);
            dialog->findChild<QSpinBox*>("processTopLayers")->setValue(2);
            dialog->findChild<QComboBox*>("processPattern")->setCurrentIndex(2);
            dialog->findChild<QComboBox*>("processSupportMode")->setCurrentIndex(1);
            auto screenshots=qEnvironmentVariable("FIVEAXIS_SCREENSHOTS");
            if(!screenshots.isEmpty()) {
                auto tabs=dialog->findChild<QTabWidget*>(); tabs->setCurrentIndex(3); QApplication::processEvents();
                dialog->grab().save(screenshots+"/process-support-settings.png");
                tabs->setCurrentIndex(0); QApplication::processEvents();
                dialog->grab().save(screenshots+"/process-quality-settings.png");
            }
            auto buttons=dialog->findChild<QDialogButtonBox*>(); buttons->button(QDialogButtonBox::Ok)->click();
        });
        settings->click(); QVERIFY(dialogVisited);
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
        auto sliders = window.findChildren<QSlider *>();
        QCOMPARE(sliders.size(), 1);
        auto timeline = sliders.front();
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
