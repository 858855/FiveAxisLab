#pragma once
#include "core/types.h"
#include "viewport.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include <QSlider>
#include <QTextEdit>
#include <QTimer>
#include <set>
struct WorkResult {
    fiveaxis::Project project;
    QString error;
    bool preview = false;
};
class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    MainWindow();
    ~MainWindow();
    void load(const QString &filename);
    void capture(const QString &filename);
    const fiveaxis::Project &current_project() const { return project; }

  private:
    fiveaxis::Project project, replay;
    Viewport *viewport;
    QComboBox *controller, *layout, *mode, *selection, *supportMode;
    QDoubleSpinBox *layerHeight, *lineWidth, *infillDensity, *speed;
    QCheckBox *curved, *dummyToolhead, *extension;
    QLabel *summary, *validation;
    QTextEdit *log;
    QProgressBar *progress;
    QSlider *timeline;
    QTimer timer;
    QElapsedTimer elapsed;
    QFutureWatcher<WorkResult> worker;
    std::shared_ptr<std::atomic_bool> cancelled;
    std::set<int> fixedFaces, forceFaces;
    bool playing = false, replaying = false;
    double time = 0;
    QString workerFingerprint;
    void execute(const QString &label, std::function<void(fiveaxis::Project &, std::atomic_bool *)> task);
    void refresh();
    void invalidate();
    void run(const QString &operation);
    void import_part();
    void import_step();
    void machine_setup();
    void machine_obstacles();
    void import_fem();
    void process_settings();
    void import_orca_settings();
    void setup_loads();
    void export_file();
    void toggle_play();
    void seek(double t);
    void save();
    void show_error(const QString &message);
};
