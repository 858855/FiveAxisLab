#include "main_window.h"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QSurfaceFormat>
#include <QTimer>
int main(int argc, char **argv) {
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setSamples(4);
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    QSurfaceFormat::setDefaultFormat(format);
    QApplication app(argc, argv);
    app.setApplicationName("FiveAxisLab");
    app.setOrganizationName("FiveAxisLab");
    app.setStyle("Fusion");
    app.setStyleSheet("QWidget{background:#141e2a;color:#dce7f3;font-family:'Segoe "
                      "UI';font-size:12px;}QGroupBox{border:1px solid "
                      "#304052;border-radius:8px;margin-top:10px;padding:9px;}QGroupBox::title{subcontrol-"
                      "origin:margin;left:10px;color:#98b3cd;}QPushButton,QComboBox,QDoubleSpinBox,QLineEdit{"
                      "background:#243246;border:1px solid "
                      "#3a4d64;border-radius:5px;padding:7px;}QPushButton:hover{background:#31445d;}"
                      "QTextEdit{background:#101822;border:1px solid "
                      "#304052;border-radius:6px;}QScrollArea{border:none;}QProgressBar{border:1px solid "
                      "#304052;}QProgressBar::chunk{background:#18b39e;}QSlider::groove:horizontal{height:"
                      "5px;background:#304052;}QSlider::handle:horizontal{width:14px;margin:-5px "
                      "0;border-radius:7px;background:#21bda5;}QMenuBar{background:#101822;}QTableWidget{"
                      "gridline-color:#304052;}");
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addOption({"project", "Open a saved project", "path"});
    parser.addOption({"capture", "Capture a native UI image then exit", "path"});
    parser.process(app);
    MainWindow window;
    if (parser.isSet("project"))
        window.load(parser.value("project"));
    window.show();
    if (parser.isSet("capture"))
        QTimer::singleShot(2000, &window, [&] {
            window.capture(parser.value("capture"));
            app.quit();
        });
    return app.exec();
}
