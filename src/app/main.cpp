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
