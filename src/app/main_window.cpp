#include "main_window.h"
#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include <QAction>
#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSet>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <algorithm>
#include <stdexcept>

using namespace fiveaxis;
class EventTimeline : public QSlider {
  public:
    EventTimeline() : QSlider(Qt::Horizontal) {}
    std::vector<double> marks;

  protected:
    void paintEvent(QPaintEvent *event) override {
        QSlider::paintEvent(event);
        QPainter painter(this);
        painter.setPen(QPen(QColor("#ff586b"), 2));
        for (double t : marks) {
            int x = 8 + int(t * (width() - 16));
            painter.drawLine(x, height() / 2 - 9, x, height() / 2 + 9);
        }
    }
};
static QPushButton *button(const QString &text, QVBoxLayout *layout, std::function<void()> task) {
    auto b = new QPushButton(text);
    layout->addWidget(b);
    QObject::connect(b, &QPushButton::clicked, b, std::move(task));
    return b;
}
static QDoubleSpinBox *spin(double value, double min, double max, int decimals = 2) {
    auto s = new QDoubleSpinBox;
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setValue(value);
    return s;
}
static QString vector_text(const Vec3 &p) {
    return QString("%1, %2, %3").arg(p.x()).arg(p.y()).arg(p.z());
}
static Vec3 parse_vector(const QString &text) {
    auto a = text.split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts);
    if (a.size() != 3)
        throw std::runtime_error("Enter three coordinates separated by commas");
    Vec3 p;
    for (int i = 0; i < 3; ++i) {
        bool ok;
        p[i] = a[i].toDouble(&ok);
        if (!ok || !std::isfinite(p[i]))
            throw std::runtime_error("Invalid coordinate");
    }
    return p;
}
MainWindow::MainWindow() : project(demo_project()) {
    setWindowTitle("FiveAxisLab — STEP collision and simulation research");
    resize(1440, 920);
    auto central = new QWidget;
    setCentralWidget(central);
    auto horizontal = new QHBoxLayout(central);
    horizontal->setContentsMargins(14, 14, 14, 14);
    horizontal->setSpacing(14);
    auto scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFixedWidth(305);
    auto panel = new QWidget;
    auto left = new QVBoxLayout(panel);
    left->setSpacing(10);
    scroll->setWidget(panel);
    horizontal->addWidget(scroll);
    auto brand = new QLabel("FiveAxisLab");
    brand->setStyleSheet("font-size:25px;font-weight:650;color:#eef5fc;padding:4px;");
    left->addWidget(brand);
    auto subtitle = new QLabel("Five-axis printing research");
    subtitle->setStyleSheet("color:#92a8bf;padding-left:4px;");
    left->addWidget(subtitle);
    auto machineGroup = new QGroupBox("Machine");
    auto mf = new QFormLayout(machineGroup);
    controller = new QComboBox;
    controller->addItems({"Duet / RepRapFirmware", "Klipper"});
    layout = new QComboBox;
    layout->addItems({"Tilting and rotating bed", "Tilting toolhead"});
    extension = new QCheckBox("Rotary-aware flow extension");
    extension->setChecked(true);
    mf->addRow("Controller", controller);
    mf->addRow("Layout", layout);
    mf->addRow(extension);
    left->addWidget(machineGroup);
    button("Machine setup / toolhead STEP", left, [this] { machine_setup(); });
    button("Import toolhead STEP", left, [this] { import_step(); });
    button("Bed, fixtures and machine bodies", left, [this] { machine_obstacles(); });
    button("Import printable part", left, [this] { import_part(); });
    auto processGroup = new QGroupBox("Print process");
    auto pf = new QFormLayout(processGroup);
    layerHeight = spin(.5, .05, 1);
    lineWidth = spin(.8, .1, 2);
    infillDensity = spin(25, 0, 100, 0);
    speed = spin(25, 1, 200, 0);
    curved = new QCheckBox("Curved layers from stress field");
    curved->setChecked(true);
    supports = new QCheckBox("Generate removable supports");
    supports->setChecked(true);
    pf->addRow("Layer height · mm", layerHeight);
    pf->addRow("Line width · mm", lineWidth);
    pf->addRow("Infill · %", infillDensity);
    pf->addRow("Speed · mm/s", speed);
    pf->addRow(curved);
    pf->addRow(supports);
    left->addWidget(processGroup);
    auto structure = new QGroupBox("Strength");
    auto sf = new QVBoxLayout(structure);
    selection = new QComboBox;
    selection->addItems({"Inspect faces", "Select fixed faces", "Select loaded faces"});
    sf->addWidget(selection);
    button("Loads, fixtures and material", sf, [this] { setup_loads(); });
    button("Solve built-in FEA", sf, [this] { run("FEA"); });
    button("Import FEM · Autodesk Fusion", sf, [this] { import_fem(); });
    button("Compare with planar", sf, [this] { run("Compare"); });
    left->addWidget(structure);
    button("Slice + validate", left, [this] {
        run("Slice");
    })->setStyleSheet("background:#087f77;font-weight:600;padding:11px;");
    button("Try collision avoidance", left, [this] { run("Repair"); });
    button("Validate current motion", left, [this] { run("Validate"); });
    button("Export G-code", left, [this] { export_file(); });
    validation = new QLabel("Motion awaits validation");
    validation->setWordWrap(true);
    left->addWidget(validation);
    summary = new QLabel;
    summary->setWordWrap(true);
    left->addWidget(summary);
    left->addStretch();
    auto right = new QVBoxLayout;
    horizontal->addLayout(right, 1);
    auto controls = new QHBoxLayout;
    mode = new QComboBox;
    mode->addItems({"Planned motion", "Exported G-code replay"});
    controls->addWidget(mode);
    auto play = new QPushButton("Play / pause");
    controls->addWidget(play);
    auto rate = new QComboBox;
    rate->addItems({"0.25×", "0.5×", "1×", "2×", "5×", "10×"});
    rate->setCurrentIndex(2);
    controls->addWidget(rate);
    auto follow = new QCheckBox("Follow nozzle");
    controls->addWidget(follow);
    auto ghost = new QCheckBox("Transparent part");
    ghost->setChecked(true);
    controls->addWidget(ghost);
    auto stressOverlay = new QCheckBox("Stress");
    controls->addWidget(stressOverlay);
    auto next = new QPushButton("Next collision");
    controls->addWidget(next);
    controls->addStretch();
    right->addLayout(controls);
    viewport = new Viewport;
    viewport->set_project(&project);
    right->addWidget(viewport, 1);
    auto transport = new QHBoxLayout;
    auto previous = new QPushButton("◀ Step");
    auto forward = new QPushButton("Step ▶");
    timeline = new EventTimeline;
    timeline->setRange(0, 10000);
    transport->addWidget(previous);
    transport->addWidget(timeline, 1);
    transport->addWidget(forward);
    right->addLayout(transport);
    log = new QTextEdit;
    log->setReadOnly(true);
    log->setMaximumHeight(140);
    right->addWidget(log);
    progress = new QProgressBar;
    progress->setRange(0, 100);
    progress->setVisible(false);
    auto cancel = new QPushButton("Cancel job");
    cancel->setVisible(false);
    statusBar()->addPermanentWidget(progress, 1);
    statusBar()->addPermanentWidget(cancel);
    auto file = menuBar()->addMenu("File");
    auto open = file->addAction("Open project…");
    open->setShortcut(QKeySequence::Open);
    connect(open, &QAction::triggered, this, [this] {
        auto f =
            QFileDialog::getOpenFileName(this, "Open research project", {}, "FiveAxisLab projects (*.5axis)");
        if (!f.isEmpty())
            load(f);
    });
    auto saveAction = file->addAction("Save project…");
    saveAction->setShortcut(QKeySequence::Save);
    connect(saveAction, &QAction::triggered, this, &MainWindow::save);
    file->addAction("Open demonstration", this, [this] {
        if (worker.isRunning())
            return;
        project = demo_project();
        fixedFaces.clear();
        forceFaces.clear();
        replaying = false;
        refresh();
        viewport->fit();
    });
    file->addAction("Export controller configuration…", this, [this] {
        auto f =
            QFileDialog::getSaveFileName(this, "Configuration template", {}, "Configuration (*.cfg *.g)");
        if (f.isEmpty())
            return;
        QFile output(f);
        if (output.open(QIODevice::WriteOnly))
            output.write(controller_template(project.machine).toUtf8());
    });
    file->addAction("Save machine profile…", this, [this] {
        auto file =
            QFileDialog::getSaveFileName(this, "Save calibrated machine", {}, "Machine profile (*.5machine)");
        if (file.isEmpty())
            return;
        try {
            Project p;
            p.machine = project.machine;
            p.toolhead = project.toolhead;
            p.fixtures = project.fixtures;
            save_project(p, file);
        } catch (const std::exception &e) {
            show_error(e.what());
        }
    });
    file->addAction("Load machine profile…", this, [this] {
        if (worker.isRunning())
            return;
        auto file =
            QFileDialog::getOpenFileName(this, "Load calibrated machine", {}, "Machine profile (*.5machine)");
        if (file.isEmpty())
            return;
        try {
            auto p = load_project(file);
            project.machine = p.machine;
            project.toolhead = p.toolhead;
            project.fixtures = p.fixtures;
            invalidate();
            if (!project.path.empty())
                run("Regenerate");
        } catch (const std::exception &e) {
            show_error(e.what());
        }
    });
    file->addAction("Exit", this, &QWidget::close);
    auto view = menuBar()->addMenu("View");
    view->addAction("Fit geometry", viewport, &Viewport::fit);
    view->addAction("Save preview image…", this, [this] {
        auto f = QFileDialog::getSaveFileName(this, "Save preview", {}, "PNG image (*.png)");
        if (!f.isEmpty())
            capture(f);
    });
    auto help = menuBar()->addMenu("Help");
    help->addAction("About and validation coverage", this, [this] {
        QMessageBox::information(
            this, "FiveAxisLab research prototype",
            "STEP geometry, machine motion and accumulated material are checked within configured "
            "tolerances.\n\nOnly modeled machine components are covered. Material strength, controller "
            "qualification and physical printing require separate validation.\n\nOrcaSlicer flow adapter: " +
                orca_revision() + "\nAGPL-3.0; see THIRD_PARTY.md.");
    });
    connect(cancel, &QPushButton::clicked, this, [this] {
        if (cancelled)
            cancelled->store(true);
    });
    connect(play, &QPushButton::clicked, this, &MainWindow::toggle_play);
    connect(follow, &QCheckBox::toggled, viewport, &Viewport::set_follow);
    connect(ghost, &QCheckBox::toggled, viewport, &Viewport::set_transparency);
    connect(stressOverlay, &QCheckBox::toggled, viewport, &Viewport::set_stress);
    connect(timeline, &QSlider::valueChanged, this, [this](int value) {
        auto &p = replaying ? replay : project;
        if (!p.path.empty())
            seek(p.path.back().time * value / 10000.0);
    });
    connect(previous, &QPushButton::clicked, this, [this] {
        auto &p = replaying ? replay : project;
        auto s = simulation_at(p, time);
        if (!p.path.empty())
            seek(p.path[std::max(0, s.move - 1)].time);
    });
    connect(forward, &QPushButton::clicked, this, [this] {
        auto &p = replaying ? replay : project;
        auto s = simulation_at(p, time);
        if (!p.path.empty())
            seek(p.path[std::min<int>(p.path.size() - 1, s.move + 1)].time);
    });
    connect(next, &QPushButton::clicked, this, [this] {
        auto &p = replaying ? replay : project;
        for (auto &event : p.report.events)
            if (event.time > time + 1e-4) {
                seek(event.time);
                log->append(event.first + " ↔ " + event.second + ": " + event.reason);
                return;
            }
        if (!p.report.events.empty())
            seek(p.report.events.front().time);
    });
    connect(&timer, &QTimer::timeout, this, [this, rate] {
        if (!playing)
            return;
        static const double rates[] = {.25, .5, 1, 2, 5, 10};
        double dt = elapsed.restart() / 1000.0;
        auto &p = replaying ? replay : project;
        if (p.path.empty())
            return;
        double next = time + dt * rates[rate->currentIndex()];
        seek(next);
        if (next >= p.path.back().time)
            playing = false;
    });
    timer.start(33);
    connect(controller, &QComboBox::currentIndexChanged, this, [this](int index) {
        project.machine.controller = Controller(index);
        invalidate();
        if (!project.path.empty())
            run("Regenerate");
    });
    connect(layout, &QComboBox::currentIndexChanged, this, [this](int index) {
        project.machine.layout = Layout(index);
        project.machine.minB = index ? -75 * pi / 180 : -pi;
        project.machine.maxB = index ? 75 * pi / 180 : pi;
        invalidate();
        if (!project.path.empty())
            run("Regenerate");
    });
    connect(extension, &QCheckBox::toggled, this, [this](bool v) {
        project.machine.klipperExtension = v;
        invalidate();
    });
    auto settings = [this] {
        project.process.layerHeight = layerHeight->value();
        project.process.width = lineWidth->value();
        project.process.infill = infillDensity->value() / 100;
        project.process.speed = speed->value();
        project.process.curved = curved->isChecked();
        project.process.supports = supports->isChecked();
        project.invalidate();
        refresh();
    };
    for (auto *s : {layerHeight, lineWidth, infillDensity, speed})
        connect(s, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [settings](double) { settings(); });
    connect(curved, &QCheckBox::toggled, this, [settings](bool) { settings(); });
    connect(supports, &QCheckBox::toggled, this, [settings](bool) { settings(); });
    connect(viewport, &Viewport::face_picked, this, [this](int face) {
        if (selection->currentIndex() == 1) {
            if (!fixedFaces.erase(face))
                fixedFaces.insert(face);
        } else if (selection->currentIndex() == 2) {
            if (!forceFaces.erase(face))
                forceFaces.insert(face);
        }
        viewport->set_selected(fixedFaces, forceFaces);
        statusBar()->showMessage(
            QString("%1 fixed faces · %2 loaded faces").arg(fixedFaces.size()).arg(forceFaces.size()));
    });
    connect(mode, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (!index) {
            replaying = false;
            viewport->set_project(&project);
            seek(0);
            return;
        }
        execute("Exported replay", [](Project &p, std::atomic_bool *cancel) {
            p = replay_gcode(p, export_gcode(p));
            p.report = validate_motion(p, cancel);
        });
    });
    connect(&worker, &QFutureWatcher<WorkResult>::finished, this, [this, cancel] {
        progress->setVisible(false);
        cancel->setVisible(false);
        auto result = worker.result();
        if (!result.error.isEmpty()) {
            show_error(result.error);
            if (result.preview)
                mode->setCurrentIndex(0);
        } else if (project.fingerprint() != workerFingerprint)
            log->append("Settings changed during the job; discarded its stale result.");
        else if (result.preview) {
            if (mode->currentIndex() == 1) {
                replay = std::move(result.project);
                replaying = true;
                viewport->set_project(&replay);
                seek(0);
            }
        } else {
            project = std::move(result.project);
            replaying = false;
            mode->setCurrentIndex(0);
            for (auto &s : project.diagnostics)
                log->append(s);
            refresh();
        }
        statusBar()->showMessage("Job finished", 4000);
    });
    connect(&worker, &QFutureWatcher<WorkResult>::started, this, [this, cancel] {
        progress->setVisible(true);
        progress->setRange(0, 0);
        cancel->setVisible(true);
    });
    log->append("Open a part and STEP toolhead, or use the synthetic reference machine. Select faces for FEA "
                "loads and fixtures. Validation is required before export.");
    refresh();
}
MainWindow::~MainWindow() {
    if (cancelled)
        cancelled->store(true);
    worker.waitForFinished();
}
void MainWindow::show_error(const QString &message) {
    log->append("Error: " + message);
    statusBar()->showMessage(message, 12000);
}
void MainWindow::refresh() {
    QSignalBlocker a(controller), b(layout), c(extension), d(layerHeight), e(lineWidth), f(infillDensity),
        g(speed), h(curved), i(supports);
    controller->setCurrentIndex(int(project.machine.controller));
    layout->setCurrentIndex(int(project.machine.layout));
    extension->setChecked(project.machine.klipperExtension);
    extension->setEnabled(project.machine.controller == Controller::Klipper);
    layerHeight->setValue(project.process.layerHeight);
    lineWidth->setValue(project.process.width);
    infillDensity->setValue(project.process.infill * 100);
    speed->setValue(project.process.speed);
    curved->setChecked(project.process.curved);
    supports->setChecked(project.process.supports);
    viewport->set_project(&project);
    viewport->set_selected(fixedFaces, forceFaces);
    auto events = static_cast<EventTimeline *>(timeline);
    events->marks.clear();
    if (!project.path.empty() && project.path.back().time > 0)
        for (const auto &event : project.report.events)
            events->marks.push_back(event.time / project.path.back().time);
    events->update();
    summary->setText(QString("%1 toolhead components\n%2 movements · %3 s\nFEM: %4")
                         .arg(project.toolhead.components.size())
                         .arg(project.path.size())
                         .arg(project.path.empty() ? 0 : project.path.back().time, 0, 'f', 1)
                         .arg(project.stress.directional      ? "Complete stress tensors"
                              : project.stress.scalar.empty() ? "No results"
                                                              : "Scalar visualization only"));
    bool current = project.report.exportable() && project.report.fingerprint == project.fingerprint();
    validation->setText(
        current
            ? QString("Motion validated\nMinimum clearance: %1 mm\n%2 checked intervals")
                  .arg(project.report.minimumClearance, 0, 'f', 3)
                  .arg(project.report.checkedIntervals)
            : QString("Export blocked\n%1 collision / unresolved events").arg(project.report.events.size()));
    validation->setStyleSheet(current ? "color:#59d5b7;" : "color:#eeb862;");
}
void MainWindow::invalidate() {
    bool valid = project.depositionValid;
    project.invalidate();
    project.depositionValid = valid;
    replaying = false;
    refresh();
}
void MainWindow::execute(const QString &label, std::function<void(Project &, std::atomic_bool *)> task) {
    if (worker.isRunning()) {
        show_error("Wait for the current job or cancel it first");
        return;
    }
    playing = false;
    cancelled = std::make_shared<std::atomic_bool>(false);
    workerFingerprint = project.fingerprint();
    Project copy = project;
    auto token = cancelled;
    statusBar()->showMessage(label);
    worker.setFuture(
        QtConcurrent::run([copy = std::move(copy), token, task = std::move(task), label]() mutable {
            WorkResult result;
            result.preview = label == "Exported replay";
            try {
                task(copy, token.get());
                result.project = std::move(copy);
            } catch (const std::exception &e) {
                result.error = e.what();
            }
            return result;
        }));
}
void MainWindow::run(const QString &operation) {
    execute(operation, [operation](Project &p, std::atomic_bool *cancel) {
        if (operation == "Slice") {
            slice_project(p, cancel);
            p.report = validate_motion(p, cancel);
            if (p.report.status == Status::Collision && !(cancel && cancel->load()))
                repair_motion(p, cancel);
        } else if (operation == "Repair") {
            repair_motion(p, cancel);
        } else if (operation == "Regenerate") {
            regenerate_joints(p);
            check_deposition(p);
            p.report = validate_motion(p, cancel);
        } else if (operation == "Compare") {
            p.comparison = compare_strength(p);
            p.diagnostics
                << QString("Dense-material surrogate compliance: planar %1, curved %2 N·mm; stress alignment "
                           "%3%. Sparse infill and physical failure are outside this constitutive model.")
                       .arg(p.comparison.planarCompliance)
                       .arg(p.comparison.curvedCompliance)
                       .arg(p.comparison.alignment * 100, 0, 'f', 1);
        } else if (operation == "FEA") {
            if (p.volume.cells.empty())
                p.volume = tetrahedralize(p.part);
            auto result = solve_fea(p.volume, p.loads);
            p.stress = result.stress;
            p.invalidate();
            p.diagnostics << QString("Linear static analysis solved. Predicted compliance: %1 N·mm.")
                                 .arg(result.compliance);
        } else
            p.report = validate_motion(p, cancel);
    });
}
void MainWindow::import_part() {
    auto file = QFileDialog::getOpenFileName(this, "Import printable geometry", {},
                                             "Meshes and CAD (*.stl *.obj *.3mf *.step *.stp)");
    if (file.isEmpty())
        return;
    execute("Import part", [file](Project &p, std::atomic_bool *) {
        p.part = load_mesh(file);
        Vec3 offset = p.part.bounds().center();
        offset.z() = p.part.bounds().min().z();
        for (auto &v : p.part.vertices)
            v -= offset;
        p.partPlacement = translation(-offset);
        p.volume = {};
        p.stress = {};
        p.loads.fixedNodes.clear();
        p.loads.nodalForces.clear();
        p.path.clear();
        p.invalidate();
    });
    fixedFaces.clear();
    forceFaces.clear();
}
void MainWindow::import_step() {
    auto file = QFileDialog::getOpenFileName(this, "Import complete toolhead assembly", {},
                                             "STEP assemblies (*.step *.stp)");
    if (file.isEmpty())
        return;
    execute("Import STEP assembly", [file](Project &p, std::atomic_bool *cancel) {
        p.toolhead = load_step_assembly(file, .02, cancel);
        p.invalidate();
        p.diagnostics
            << "Toolhead imported. Open Machine setup to calibrate the nozzle tip, axis and moving links.";
    });
}
void MainWindow::machine_setup() {
    if (worker.isRunning())
        return;
    QDialog dialog(this);
    dialog.setWindowTitle("Machine setup · STEP toolhead");
    dialog.resize(1000, 720);
    auto horizontal = new QHBoxLayout(&dialog);
    auto formWidget = new QWidget;
    auto left = new QVBoxLayout(formWidget);
    auto form = new QFormLayout;
    left->addLayout(form);
    Project copy = project;
    left->addWidget(new QLabel(
        "Source STEP length units: " +
        (copy.toolhead.sourceUnits.isEmpty() ? QString("synthetic mm") : copy.toolhead.sourceUnits) +
        " · converted to mm"));
    auto tcp = new QLineEdit(vector_text(copy.toolhead.tcp)),
         axis = new QLineEdit(vector_text(copy.toolhead.axis)),
         mount = new QLineEdit(vector_text(copy.toolhead.mount.translation())),
         bed = new QLineEdit(vector_text(copy.machine.bedPivot)),
         pa = new QLineEdit(vector_text(copy.machine.headPivotA)),
         pb = new QLineEdit(vector_text(copy.machine.headPivotB)),
         minimum = new QLineEdit(vector_text(copy.machine.minXYZ)),
         maximum = new QLineEdit(vector_text(copy.machine.maxXYZ));
    auto clearance = spin(copy.machine.clearance, 0, 10, 3),
         radius = spin(copy.toolhead.contactRadius, .01, 5, 3),
         height = spin(copy.toolhead.contactHeight, .01, 5, 3);
    form->addRow("Nozzle tip · CAD mm", tcp);
    form->addRow("Axis · tip toward toolhead", axis);
    form->addRow("Mount translation · mm", mount);
    form->addRow("Bed pivot · mm", bed);
    form->addRow("Head A pivot · mm", pa);
    form->addRow("Head B pivot · mm", pb);
    form->addRow("XYZ minimum · mm", minimum);
    form->addRow("XYZ maximum · mm", maximum);
    form->addRow("Required clearance · mm", clearance);
    form->addRow("Tip contact radius · mm", radius);
    form->addRow("Tip contact height · mm", height);
    auto table = new QTableWidget(int(copy.toolhead.components.size()), 4);
    table->setHorizontalHeaderLabels({"Component / instance", "Moving link", "Nozzle", "Visual only"});
    std::vector<QComboBox *> links;
    std::vector<QCheckBox *> nozzles, visuals;
    for (size_t i = 0; i < copy.toolhead.components.size(); ++i) {
        auto &c = copy.toolhead.components[i];
        table->setItem(i, 0, new QTableWidgetItem(c.name + " · " + c.id));
        auto link = new QComboBox;
        link->addItems({"Carriage", "Rotary stage", "Nozzle assembly"});
        link->setCurrentIndex(int(c.link));
        links.push_back(link);
        table->setCellWidget(i, 1, link);
        auto nozzle = new QCheckBox;
        nozzle->setChecked(c.nozzle);
        nozzles.push_back(nozzle);
        table->setCellWidget(i, 2, nozzle);
        auto visual = new QCheckBox;
        visual->setChecked(c.visualOnly);
        visuals.push_back(visual);
        table->setCellWidget(i, 3, visual);
    }
    table->resizeColumnsToContents();
    left->addWidget(table, 1);
    auto interfaces = new QTextEdit;
    interfaces->setMaximumHeight(75);
    interfaces->setPlaceholderText("First id | second id | CAD center x,y,z | box size x,y,z");
    for (const auto &rule : copy.toolhead.interfaces)
        interfaces->append(rule.first + " | " + rule.second + " | " + vector_text(rule.center) + " | " +
                           vector_text(rule.size));
    left->addWidget(new QLabel("Bounded assembly interfaces · first component CAD frame"));
    left->addWidget(interfaces);
    auto buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    left->addWidget(buttons);
    horizontal->addWidget(formWidget, 1);
    auto previewColumn = new QVBoxLayout;
    auto preview = new Viewport;
    previewColumn->addWidget(preview, 1);
    horizontal->addLayout(previewColumn, 1);
    auto pick = new QPushButton("Pick nozzle tip in preview");
    previewColumn->addWidget(pick);
    auto angleA = spin(0, -75, 75, 1), angleB = spin(0, -180, 180, 1);
    previewColumn->addWidget(new QLabel("Calibration pose · A / B or C degrees"));
    previewColumn->addWidget(angleA);
    previewColumn->addWidget(angleB);
    copy.path.clear();
    Sample pose;
    pose.joint.xyz = Vec3(0, 0, 15);
    copy.path.push_back(pose);
    preview->set_project(&copy);
    preview->fit();
    connect(pick, &QPushButton::clicked, preview, [preview] { preview->set_tool_pick(true); });
    connect(preview, &Viewport::tool_picked, &dialog, [tcp, nozzles](int index, Vec3 point) {
        tcp->setText(vector_text(point));
        for (size_t i = 0; i < nozzles.size(); ++i)
            nozzles[i]->setChecked(int(i) == index);
    });
    auto updatePose = [&] {
        try {
            copy.toolhead.tcp = parse_vector(tcp->text());
            auto n = parse_vector(axis->text());
            if (n.norm() < 1e-8)
                return;
            copy.toolhead.axis = n.normalized();
            copy.toolhead.mount = translation(parse_vector(mount->text()));
            copy.toolhead.mount.linear() =
                Eigen::Quaterniond::FromTwoVectors(copy.toolhead.axis, Vec3::UnitZ()).toRotationMatrix();
            copy.machine.bedPivot = parse_vector(bed->text());
            copy.machine.headPivotA = parse_vector(pa->text());
            copy.machine.headPivotB = parse_vector(pb->text());
            for (size_t i = 0; i < links.size(); ++i)
                copy.toolhead.components[i].link = Link(links[i]->currentIndex());
        } catch (...) {
            return;
        }
        copy.path.front().joint.a = angleA->value() * pi / 180;
        copy.path.front().joint.b = angleB->value() * pi / 180;
        preview->set_time(0);
    };
    connect(angleA, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [&](double) { updatePose(); });
    connect(angleB, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [&](double) { updatePose(); });
    for (auto field : {tcp, axis, mount, bed, pa, pb})
        connect(field, &QLineEdit::textChanged, &dialog, [&](const QString &) { updatePose(); });
    for (auto field : links)
        connect(field, &QComboBox::currentIndexChanged, &dialog, [&](int) { updatePose(); });
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        try {
            ToolheadAssembly assembly = project.toolhead;
            assembly.tcp = parse_vector(tcp->text());
            assembly.axis = parse_vector(axis->text());
            if (!assembly.axis.allFinite() || assembly.axis.norm() < 1e-8)
                throw std::runtime_error("Nozzle axis cannot be zero");
            assembly.axis.normalize();
            assembly.mount = translation(parse_vector(mount->text()));
            assembly.mount.linear() =
                Eigen::Quaterniond::FromTwoVectors(assembly.axis, Vec3::UnitZ()).toRotationMatrix();
            assembly.contactRadius = radius->value();
            assembly.contactHeight = height->value();
            int nozzleCount = 0;
            for (size_t i = 0; i < assembly.components.size(); ++i) {
                assembly.components[i].link = Link(links[i]->currentIndex());
                assembly.components[i].nozzle = nozzles[i]->isChecked();
                assembly.components[i].visualOnly = visuals[i]->isChecked();
                if (nozzles[i]->isChecked())
                    ++nozzleCount;
            }
            if (nozzleCount != 1)
                throw std::runtime_error("Select exactly one nozzle component");
            assembly.interfaces.clear();
            for (auto line : interfaces->toPlainText().split('\n', Qt::SkipEmptyParts)) {
                auto fields = line.split('|');
                if (fields.size() != 4)
                    throw std::runtime_error("Interface: first id | second id | center | box size");
                ToolheadAssembly::InterfaceAllowance rule;
                rule.first = fields[0].trimmed();
                rule.second = fields[1].trimmed();
                rule.center = parse_vector(fields[2]);
                rule.size = parse_vector(fields[3]);
                if (rule.size.minCoeff() <= 0)
                    throw std::runtime_error("Positive interface dimensions required");
                assembly.interfaces.push_back(rule);
            }
            auto min = parse_vector(minimum->text()), max = parse_vector(maximum->text());
            if ((max - min).minCoeff() <= 0)
                throw std::runtime_error("XYZ maximum must exceed minimum");
            project.toolhead = std::move(assembly);
            project.machine.bedPivot = parse_vector(bed->text());
            project.machine.headPivotA = parse_vector(pa->text());
            project.machine.headPivotB = parse_vector(pb->text());
            project.machine.minXYZ = parse_vector(minimum->text());
            project.machine.maxXYZ = parse_vector(maximum->text());
            project.machine.clearance = clearance->value();
            invalidate();
            dialog.accept();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Setup", e.what());
        }
    });
    if (dialog.exec() == QDialog::Accepted && !project.path.empty())
        run("Regenerate");
}
void MainWindow::machine_obstacles() {
    if (worker.isRunning())
        return;
    Project copy = project;
    QDialog dialog(this);
    dialog.setWindowTitle("Bed, fixtures and machine bodies");
    dialog.resize(720, 480);
    auto box = new QVBoxLayout(&dialog);
    auto list = new QTableWidget(0, 2);
    list->setHorizontalHeaderLabels({"Physical body", "Attachment"});
    box->addWidget(list, 1);
    auto rebuild = [&] {
        list->setRowCount(int(copy.fixtures.size()));
        for (size_t i = 0; i < copy.fixtures.size(); ++i) {
            list->setItem(int(i), 0, new QTableWidgetItem(copy.fixtures[i].name));
            auto attachment = new QComboBox;
            attachment->addItems({"Machine world", "Workpiece / rotary bed"});
            attachment->setCurrentIndex(copy.fixtures[i].movesWithBed ? 1 : 0);
            list->setCellWidget(int(i), 1, attachment);
        }
        list->resizeColumnsToContents();
    };
    rebuild();
    auto form = new QFormLayout;
    box->addLayout(form);
    auto name = new QLineEdit("Fixture"), center = new QLineEdit("0, 0, -2"),
         size = new QLineEdit("100, 100, 4");
    form->addRow("Name (bed bodies must include Bed)", name);
    form->addRow("Center · mm", center);
    form->addRow("Box dimensions · mm", size);
    button("Add box", box, [&] {
        try {
            auto dims = parse_vector(size->text());
            if (dims.minCoeff() <= 0)
                throw std::runtime_error("Positive dimensions required");
            copy.fixtures.push_back(
                {name->text(), box_mesh(parse_vector(center->text()), dims, name->text()), true});
            rebuild();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Geometry", e.what());
        }
    });
    button("Import STEP fixture or machine body", box, [&] {
        auto file = QFileDialog::getOpenFileName(&dialog, "STEP machine body", {}, "STEP (*.step *.stp)");
        if (file.isEmpty())
            return;
        try {
            auto model = load_step_assembly(file);
            auto shift = parse_vector(center->text());
            for (auto c : model.components) {
                for (auto &v : c.mesh.vertices)
                    v += shift;
                Fixture f{c.name, c.mesh, false};
                f.sourceStep = model.sourceStep;
                copy.fixtures.push_back(std::move(f));
            }
            rebuild();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "STEP", e.what());
        }
    });
    button("Remove selected body", box, [&] {
        int row = list->currentRow();
        if (row >= 0) {
            copy.fixtures.erase(copy.fixtures.begin() + row);
            rebuild();
        }
    });
    auto actions = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    box->addWidget(actions);
    connect(actions, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(actions, &QDialogButtonBox::accepted, &dialog, [&] {
        for (size_t i = 0; i < copy.fixtures.size(); ++i) {
            copy.fixtures[i].name = list->item(int(i), 0)->text();
            copy.fixtures[i].movesWithBed =
                static_cast<QComboBox *>(list->cellWidget(int(i), 1))->currentIndex() == 1;
        }
        project.fixtures = std::move(copy.fixtures);
        invalidate();
        dialog.accept();
    });
    dialog.exec();
}
void MainWindow::import_fem() {
    auto files = QFileDialog::getOpenFileNames(this, "Fusion VTU · one tensor file or six component files",
                                               {}, "VTU results (*.vtu)");
    if (files.isEmpty())
        return;
    try {
        if (files.size() != 1 && files.size() != 6)
            throw std::runtime_error("Choose one tensor/scalar file or exactly six component files");
        auto info = inspect_vtu(files.front());
        QDialog dialog(this);
        dialog.setWindowTitle("Autodesk Fusion FEM import");
        auto layout = new QFormLayout(&dialog);
        auto location = new QComboBox;
        location->addItems({"Point results", "Cell / element results"});
        auto arrays = new QComboBox;
        arrays->addItems(info.pointArrays);
        auto length = spin(1, .000001, 100000, 6), stress = spin(1, .000001, 1000000000, 6);
        auto step = new QLineEdit("static");
        layout->addRow("Result association", location);
        layout->addRow("Tensor or scalar array", arrays);
        layout->addRow("Length → mm multiplier", length);
        layout->addRow("Stress → MPa multiplier", stress);
        layout->addRow("Study / result step", step);
        std::array<QLineEdit *, 6> mappings;
        std::array<QComboBox *, 6> componentFiles{};
        const char *labels[] = {"XX", "YY", "ZZ", "XY", "YZ", "ZX"};
        for (int i = 0; i < 6; ++i) {
            mappings[i] = new QLineEdit;
            mappings[i]->setPlaceholderText("Optional scalar array name");
            layout->addRow(QString(labels[i]) + " component", mappings[i]);
            if (files.size() == 6) {
                componentFiles[i] = new QComboBox;
                componentFiles[i]->addItems(files);
                componentFiles[i]->setCurrentIndex(i);
                layout->addRow(QString(labels[i]) + " source file", componentFiles[i]);
            }
        }
        auto offset = new QLineEdit(vector_text(project.partPlacement.translation())),
             rotation = new QLineEdit("0, 0, 0");
        layout->addRow("Alignment translation · mm", offset);
        layout->addRow("Alignment XYZ rotation · °", rotation);
        if (files.size() > 1)
            layout->addRow(new QLabel(
                "Assign each stress component to its source file.\nMesh and node associations must match."));
        connect(location, &QComboBox::currentIndexChanged, &dialog, [&](int i) {
            arrays->clear();
            arrays->addItems(i ? info.cellArrays : info.pointArrays);
        });
        auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        layout->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        if (dialog.exec() != QDialog::Accepted)
            return;
        if (files.size() == 6) {
            QStringList ordered;
            for (auto field : componentFiles)
                ordered << field->currentText();
            if (QSet<QString>(ordered.begin(), ordered.end()).size() != 6)
                throw std::runtime_error("Each component must use a different source file");
            files = ordered;
        }
        VtuOptions options;
        options.pointData = location->currentIndex() == 0;
        options.array = arrays->currentText();
        options.lengthScale = length->value();
        options.stressScale = stress->value();
        options.resultStep = step->text();
        Vec3 r = parse_vector(rotation->text()) * pi / 180;
        options.alignment = translation(parse_vector(offset->text())) * rotation_z(r.z()) *
                            rotation_y(r.y()) * rotation_x(r.x());
        for (int i = 0; i < 6; ++i)
            options.components[i] = mappings[i]->text();
        execute("Import Fusion FEM", [files, options](Project &p, std::atomic_bool *) {
            p.stress = import_vtu(files, options);
            p.invalidate();
            p.diagnostics << QString("Imported FEM: %1 nodes, %2 cells, %3.")
                                 .arg(p.stress.mesh.vertices.size())
                                 .arg(p.stress.mesh.cells.size())
                                 .arg(p.stress.directional ? "full stress tensor"
                                                           : "scalar visualization only");
        });
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::setup_loads() {
    try {
        if (fixedFaces.empty() || forceFaces.empty())
            throw std::runtime_error("Select fixed and loaded part faces in the preview first");
        if (project.volume.cells.empty())
            project.volume = tetrahedralize(project.part);
        QDialog dialog(this);
        dialog.setWindowTitle("Loads and material");
        auto form = new QFormLayout(&dialog);
        auto force = new QLineEdit("0, 0, -10");
        auto young = spin(project.loads.material.young, 1, 1000000, 1),
             poisson = spin(project.loads.material.poisson, -.9, .49, 3),
             transverse = spin(project.loads.material.transverseYoung, 1, 1000000, 1);
        form->addRow("Total applied force · N", force);
        form->addRow("Young's modulus · MPa", young);
        form->addRow("Poisson ratio", poisson);
        form->addRow("Transverse modulus · MPa", transverse);
        auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        form->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        if (dialog.exec() != QDialog::Accepted)
            return;
        auto nodes = [&](const std::set<int> &faces) {
            std::set<int> found;
            for (size_t i = 0; i < project.volume.vertices.size(); ++i)
                for (int face : faces) {
                    auto t = project.part.triangles.at(face);
                    Vec3 a = project.part.vertices[t[0]], u = project.part.vertices[t[1]] - a,
                         v = project.part.vertices[t[2]] - a, w = project.volume.vertices[i] - a,
                         n = u.cross(v);
                    if (n.norm() < 1e-8 || std::abs(w.dot(n.normalized())) > 1e-5)
                        continue;
                    Eigen::Matrix2d M;
                    M << u.dot(u), u.dot(v), u.dot(v), v.dot(v);
                    Eigen::Vector2d b = M.inverse() * Eigen::Vector2d(w.dot(u), w.dot(v));
                    if (b.minCoeff() >= -1e-6 && b.sum() <= 1 + 1e-6)
                        found.insert(i);
                }
            return found;
        };
        auto fixed = nodes(fixedFaces), loaded = nodes(forceFaces);
        if (fixed.size() < 3 || loaded.empty())
            throw std::runtime_error("Selected faces do not map to sufficient volume nodes");
        project.loads.fixedNodes.assign(fixed.begin(), fixed.end());
        project.loads.nodalForces.clear();
        Vec3 f = parse_vector(force->text()) / double(loaded.size());
        for (int i : loaded)
            project.loads.nodalForces.emplace_back(i, f);
        project.loads.material = {young->value(), poisson->value(), transverse->value()};
        project.stress = {};
        project.invalidate();
        refresh();
        log->append(
            QString("Mapped %1 fixed nodes and %2 loaded nodes.").arg(fixed.size()).arg(loaded.size()));
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::export_file() {
    try {
        auto text = export_gcode(project);
        auto f =
            QFileDialog::getSaveFileName(this, "Export validated research G-code", {}, "G-code (*.gcode)");
        if (f.isEmpty())
            return;
        QFile file(f);
        if (!file.open(QIODevice::WriteOnly))
            throw std::runtime_error("Cannot write G-code");
        file.write(text.toUtf8());
        log->append("Exported " + f);
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::save() {
    auto f = QFileDialog::getSaveFileName(this, "Save research project", {}, "FiveAxisLab project (*.5axis)");
    if (f.isEmpty())
        return;
    try {
        save_project(project, f);
        log->append("Saved " + f);
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::load(const QString &filename) {
    try {
        project = load_project(filename);
        fixedFaces.clear();
        forceFaces.clear();
        replaying = false;
        refresh();
        viewport->fit();
        seek(0);
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::toggle_play() {
    auto &p = replaying ? replay : project;
    if (p.path.empty())
        return;
    if (time >= p.path.back().time)
        seek(0);
    playing = !playing;
    elapsed.restart();
}
void MainWindow::seek(double t) {
    auto &p = replaying ? replay : project;
    if (p.path.empty())
        return;
    time = std::clamp(t, 0.0, p.path.back().time);
    QSignalBlocker blocker(timeline);
    timeline->setValue(p.path.back().time > 0 ? int(time / p.path.back().time * 10000) : 0);
    viewport->set_time(time);
}
void MainWindow::capture(const QString &filename) {
    seek(project.path.empty() ? 0 : project.path.back().time * .65);
    viewport->repaint();
    grab().save(filename);
}
