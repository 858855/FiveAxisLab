#include "app/main_window.h"
#include "core/geometry.h"
#include "core/project.h"
#include <QPushButton>
#include <QSurfaceFormat>
#include <QTemporaryDir>
#include <QtTest/QtTest>

using namespace fiveaxis;
class DesktopTests : public QObject {
    Q_OBJECT
  private Q_SLOTS:
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
