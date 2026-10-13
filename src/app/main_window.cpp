#include "main_window.h"
#include "core/collision.h"
#include "core/controllers.h"
#include "core/geometry.h"
#include "core/machine_settings.h"
#include "core/motion_preview.h"
#include "core/nonplanar.h"
#include "core/plate.h"
#include "core/process.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/slicing.h"
#include "core/step_import.h"
#include "core/stress.h"
#include "machine_dialog.h"
#include "plate_editor.h"
#include "step_dialog.h"
#include <QAction>
#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSet>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QStandardItemModel>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
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
    std::vector<double> marks, changes;

  protected:
    void paintEvent(QPaintEvent *event) override {
        QSlider::paintEvent(event);
        QPainter painter(this);
        painter.setPen(QPen(QColor("#6699df"), 2));
        for (double t : changes) {
            int x = 8 + int(t * (width() - 16));
            painter.drawLine(x, height() / 2 - 5, x, height() / 2 + 5);
        }
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
    setWindowTitle("FiveAxisLab — five-axis research slicer");
    resize(1600, 980);
    setStyleSheet(
        "QWidget{font-family:'Segoe UI';font-size:9pt;color:#353d40;background:#fafafa;}"
        "QGroupBox{border:1px solid #dce0e1;border-radius:3px;margin-top:9px;padding-top:5px;}"
        "QGroupBox::title{subcontrol-origin:margin;left:7px;font-weight:600;}"
        "QPushButton,QComboBox,QLineEdit,QSpinBox,QDoubleSpinBox{background:white;border:1px solid "
        "#d6dbdd;border-radius:3px;padding:3px;}"
        "QPushButton:hover{border-color:#00ad55;background:#effaf3;}QWidget:disabled{color:#9b9fa2;}"
        "QScrollArea{border:0;}QTabWidget::pane{border:1px solid #dce0e1;}QTabBar::tab{padding:7px "
        "5px;border-bottom:2px solid transparent;}"
        "QTabBar::tab:selected{border-bottom-color:#00ad55;font-weight:600;}"
        "QTextEdit{background:#f2f4f4;border:1px solid #dce0e1;}"
        "QSlider::groove:horizontal{height:4px;background:#d6dcda;}QSlider::handle:horizontal{width:12px;"
        "margin:-5px 0;border-radius:6px;background:#00af55;}"
        "QSlider::groove:vertical{width:4px;background:#d6dcda;}QSlider::handle:vertical{height:12px;margin:"
        "0 -5px;border-radius:6px;background:#00af55;}"
        "QHeaderView::section{background:#eff1f1;border:0;padding:3px;font-weight:600;}QTableWidget{border:0;"
        "gridline-color:#e6e9e9;}"
        "QCheckBox{spacing:5px;}QProgressBar::chunk{background:#00ad55;}");
    auto central = new QWidget;
    setCentralWidget(central);
    auto vertical = new QVBoxLayout(central);
    vertical->setContentsMargins(0, 0, 0, 0);
    vertical->setSpacing(0);
    auto navigation = new QWidget;
    navigation->setObjectName("workspaceNavigation");
    navigation->setStyleSheet(
        "#workspaceNavigation{background:#394345;}#workspaceNavigation "
        "QLabel{background:transparent;color:white;font-size:15px;font-weight:600;}QTabBar::tab{background:#"
        "394345;color:white;padding:10px "
        "18px;}QTabBar::tab:selected{background:#00ad55;border:0;}QPushButton{padding:7px 15px;}");
    auto nav = new QHBoxLayout(navigation);
    nav->setContentsMargins(12, 0, 10, 0);
    nav->addWidget(new QLabel("FiveAxisLab"));
    workspaces = new QTabBar;
    workspaces->setObjectName("workspaceTabs");
    workspaces->addTab("Prepare");
    workspaces->addTab("Preview");
    nav->addWidget(workspaces);
    auto machineButton = new QPushButton("Machine setup");
    nav->addWidget(machineButton);
    auto plateButton = new QPushButton("Plate / tools");
    plateButton->setObjectName("plateTools");
    nav->addWidget(plateButton);
    connect(plateButton, &QPushButton::clicked, this, &MainWindow::plate_settings);
    auto strengthButton = new QPushButton("Strength / FEM");
    nav->addWidget(strengthButton);
    nav->addStretch();
    sliceButton = new QPushButton("Slice + validate");
    sliceButton->setObjectName("slicePlate");
    sliceButton->setStyleSheet("background:#00ad55;color:white;font-weight:600;");
    nav->addWidget(sliceButton);
    exportButton = new QPushButton("Export G-code");
    exportButton->setObjectName("exportGcode");
    nav->addWidget(exportButton);
    vertical->addWidget(navigation);
    auto body = new QHBoxLayout;
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(1);
    vertical->addLayout(body, 1);
    auto panel = new QWidget;
    panel->setFixedWidth(390);
    auto left = new QVBoxLayout(panel);
    left->setContentsMargins(7, 5, 7, 5);
    left->setSpacing(5);
    body->addWidget(panel);
    auto machineGroup = new QGroupBox("Printer · machine");
    auto mf = new QFormLayout(machineGroup);
    mf->setContentsMargins(7, 12, 7, 5);
    controller = new QComboBox;
    controller->setObjectName("machineController");
    controller->addItems({"Duet / RepRapFirmware", "Klipper", "Generic / Marlin · XYZ"});
    layout = new QComboBox;
    layout->setObjectName("machineLayout");
    layout->addItems({"Tilting and rotating bed", "Tilting toolhead", "Standard XYZ · 3-axis"});
    extension = new QCheckBox("Rotary-aware flow extension");
    extension->setChecked(true);
    dummyToolhead = new QCheckBox("Generate with dummy toolhead");
    dummyToolhead->setObjectName("dummyToolhead");
    dummyToolhead->setToolTip("Use a synthetic nozzle, heat break, heater block and fan shroud. The imported "
                              "STEP assembly is retained.");
    mf->addRow("Controller", controller);
    mf->addRow("Kinematics", layout);
    mf->addRow(extension);
    mf->addRow(dummyToolhead);
    left->addWidget(machineGroup);
    auto filament = new QGroupBox("Primary filament · plate profiles in Plate / tools");
    auto ff = new QFormLayout(filament);
    ff->setContentsMargins(7, 12, 7, 5);
    filamentName = new QLineEdit;
    filamentName->setObjectName("filamentName");
    ff->addRow("Filament", filamentName);
    auto material = new QWidget;
    auto materialRow = new QHBoxLayout(material);
    materialRow->setContentsMargins(0, 0, 0, 0);
    filamentDensity = spin(1.24, .1, 5, 3);
    filamentDensity->setSuffix(" g/cm³");
    filamentDensity->setObjectName("filamentDensity");
    filamentCost = spin(25, 0, 10000, 2);
    filamentCost->setSuffix(" EUR/kg");
    filamentCost->setObjectName("filamentCost");
    materialRow->addWidget(filamentDensity);
    materialRow->addWidget(filamentCost);
    ff->addRow(material);
    left->addWidget(filament);
    auto presets = new QHBoxLayout;
    auto preset = new QComboBox;
    preset->setObjectName("processPreset");
    preset->addItems({"Custom research process", "0.30 mm · 0.6 mm nozzle · screenshot reference"});
    preset->setToolTip("The screenshot reference contains implemented settings. Machine limits still apply; "
                       "it is not a calibrated Bambu preset.");
    presets->addWidget(preset, 1);
    auto importPreset = new QPushButton("Load");
    auto savePreset = new QPushButton("Save");
    presets->addWidget(importPreset);
    presets->addWidget(savePreset);
    left->addLayout(presets);
    processEditor = new ProcessEditor;
    left->addWidget(processEditor, 1);
    // Bind the compact editor to the same fields used by project refresh.
    layerHeight = processEditor->findChild<QDoubleSpinBox *>("layerHeight");
    lineWidth = processEditor->findChild<QDoubleSpinBox *>("width");
    infillDensity = processEditor->findChild<QDoubleSpinBox *>("infill");
    speed = processEditor->findChild<QDoubleSpinBox *>("speed");
    curved = processEditor->findChild<QCheckBox *>("curved");
    supportMode = processEditor->findChild<QComboBox *>("supportMode");
    processError = new QLabel;
    processError->setObjectName("processValidationError");
    processError->setWordWrap(true);
    processError->setStyleSheet("color:#bc3030;");
    left->addWidget(processError);
    validation = new QLabel;
    validation->setWordWrap(true);
    left->addWidget(validation);
    summary = new QLabel;
    summary->setWordWrap(true);
    summary->hide();
    left->addWidget(summary);
    auto right = new QVBoxLayout;
    right->setContentsMargins(5, 5, 5, 5);
    right->setSpacing(5);
    body->addLayout(right, 1);
    auto toolbar = new QHBoxLayout;
    auto importPart = new QPushButton("Import part");
    importPart->setIcon(style()->standardIcon(QStyle::SP_FileIcon));
    toolbar->addWidget(importPart);
    auto importHead = new QPushButton("Import toolhead STEP");
    toolbar->addWidget(importHead);
    auto fitButton = new QPushButton("Fit view");
    toolbar->addWidget(fitButton);
    painting = new QCheckBox("Paint");
    painting->setObjectName("paintMaterialEnabled");
    paintMaterial = new QComboBox;
    paintMaterial->setObjectName("paintMaterial");
    paintMode = new QComboBox;
    paintMode->addItems({"Brush", "Connected fill"});
    paintRadius = spin(3, .1, 100);
    paintRadius->setSuffix(" mm");
    paintRadius->setMaximumWidth(82);
    auto undoPaint = new QPushButton("Undo paint");
    connect(undoPaint, &QPushButton::clicked, this, [this] {
        if (paintHistory.empty())
            return;
        auto previous = std::move(paintHistory.back());
        paintHistory.pop_back();
        for (size_t i = 0; i < project.objects.size() && i < previous.size(); ++i)
            project.objects[i].assignments = previous[i];
        invalidate();
        refresh();
    });
    toolbar->addWidget(painting);
    toolbar->addWidget(paintMaterial);
    toolbar->addWidget(paintMode);
    toolbar->addWidget(paintRadius);
    toolbar->addWidget(undoPaint);
    auto colorMode = new QComboBox;
    colorMode->addItems({"Color: role", "Color: material", "Color: tool", "Color: object"});
    toolbar->addWidget(colorMode);
    partName = new QLabel;
    partName->setObjectName("partObjectName");
    toolbar->addWidget(partName);
    toolbar->addStretch();
    right->addLayout(toolbar);
    auto controlsPanel = new QWidget;
    controlsPanel->setObjectName("simulationControls");
    auto controls = new QHBoxLayout(controlsPanel);
    controls->setContentsMargins(0, 0, 0, 0);
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
    right->addWidget(controlsPanel);
    auto pathsPanel = new QWidget;
    pathsPanel->setObjectName("simulationPaths");
    auto pathControls = new QHBoxLayout(pathsPanel);
    pathControls->setContentsMargins(0, 0, 0, 0);
    pathControls->addWidget(new QLabel("Toolpaths"));
    auto paths = new QComboBox;
    paths->setObjectName("toolpathMode");
    paths->addItems({"Full job", "Up to nozzle", "Current layer", "Hidden"});
    pathControls->addWidget(paths);
    auto travels = new QCheckBox("Travel paths");
    travels->setObjectName("travelPaths");
    travels->setChecked(true);
    pathControls->addWidget(travels);
    auto deposited = new QCheckBox("Deposited material");
    deposited->setChecked(true);
    pathControls->addWidget(deposited);
    pathControls->addStretch();
    right->addWidget(pathsPanel);
    viewport = new Viewport;
    connect(colorMode, &QComboBox::currentIndexChanged, viewport, &Viewport::set_color_mode);
    viewport->setObjectName("simulationViewport");
    connect(paths, &QComboBox::currentIndexChanged, viewport, &Viewport::set_path_mode);
    connect(travels, &QCheckBox::toggled, viewport, &Viewport::set_travel_paths);
    connect(deposited, &QCheckBox::toggled, viewport, &Viewport::set_deposited_visible);
    auto showHead = new QCheckBox("Toolhead");
    showHead->setChecked(true);
    showHead->setObjectName("showToolhead");
    pathControls->insertWidget(4, showHead);
    connect(showHead, &QCheckBox::toggled, viewport, &Viewport::set_toolhead_visible);
    auto axesVisible = new QCheckBox("Rotary axes");
    axesVisible->setChecked(true);
    axesVisible->setObjectName("showRotaryAxes");
    pathControls->insertWidget(5, axesVisible);
    connect(axesVisible, &QCheckBox::toggled, viewport, &Viewport::set_rotary_axes_visible);
    auto canvas = new QHBoxLayout;
    canvas->setSpacing(6);
    canvas->addWidget(viewport, 1);
    canvas->addWidget(create_preview_inspector());
    right->addLayout(canvas, 1);
    auto transportPanel = new QWidget;
    transportPanel->setObjectName("simulationTransport");
    auto transport = new QHBoxLayout(transportPanel);
    transport->setContentsMargins(0, 0, 0, 0);
    auto previous = new QPushButton("◀ Step"), forward = new QPushButton("Step ▶");
    timeline = new EventTimeline;
    timeline->setObjectName("moveTimeline");
    timeline->setRange(0, 10000);
    transport->addWidget(previous);
    transport->addWidget(timeline, 1);
    transport->addWidget(forward);
    right->addWidget(transportPanel);
    log = new QTextEdit;
    log->setReadOnly(true);
    log->setMaximumHeight(70);
    log->setObjectName("researchLog");
    right->addWidget(log);
    connect(importPart, &QPushButton::clicked, this, &MainWindow::import_part);
    connect(importHead, &QPushButton::clicked, this, &MainWindow::import_step);
    connect(fitButton, &QPushButton::clicked, viewport, &Viewport::fit);
    connect(machineButton, &QPushButton::clicked, this, &MainWindow::machine_setup);
    selection = new QComboBox;
    selection->addItems({"Inspect faces", "Select fixed faces", "Select loaded faces"});
    selection->hide();
    selection->setParent(panel);
    auto strengthMenu = new QMenu(strengthButton);
    strengthMenu->addAction("Loads, fixtures and material", this, &MainWindow::setup_loads);
    strengthMenu->addAction("Select fixed faces", this, [this] {
        selection->setCurrentIndex(1);
        statusBar()->showMessage("Click part faces to select fixtures");
    });
    strengthMenu->addAction("Select loaded faces", this, [this] {
        selection->setCurrentIndex(2);
        statusBar()->showMessage("Click part faces to apply forces");
    });
    strengthMenu->addAction("Inspect faces", this, [this] { selection->setCurrentIndex(0); });
    strengthMenu->addAction("Solve built-in FEA", this, [this] { run("FEA"); });
    strengthMenu->addAction("Import FEM · Autodesk Fusion", this, &MainWindow::import_fem);
    strengthMenu->addAction("Compare with planar", this, [this] { run("Compare"); });
    strengthButton->setMenu(strengthMenu);
    connect(sliceButton, &QPushButton::clicked, this, [this] {
        set_workspace(1);
        run("Slice");
    });
    connect(exportButton, &QPushButton::clicked, this, &MainWindow::export_file);
    connect(workspaces, &QTabBar::currentChanged, this, &MainWindow::set_workspace);
    connect(importPreset, &QPushButton::clicked, this, &MainWindow::import_orca_settings);
    connect(preset, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (index != 1 || worker.isRunning())
            return;
        QFile file(":/presets/orca-screenshots.json");
        if (!file.open(QIODevice::ReadOnly)) {
            show_error("Bundled process preset is missing");
            return;
        }
        auto next = read_process(QJsonDocument::fromJson(file.readAll()).object());
        if (next.nozzleDiameter != project.process.nozzleDiameter)
            project.dummyToolhead = make_dummy_toolhead(next.nozzleDiameter);
        project.process = next;
        project.invalidate();
        replaying = false;
        processEditor->set_settings(next);
        refresh();
    });
    connect(savePreset, &QPushButton::clicked, this, [this] {
        if (!processEditor->valid())
            return;
        auto file =
            QFileDialog::getSaveFileName(this, "Save process preset", {}, "FiveAxisLab process (*.json)");
        if (file.isEmpty())
            return;
        QFile output(file);
        if (output.open(QIODevice::WriteOnly))
            output.write(QJsonDocument(process_json(project.process)).toJson());
        else
            show_error("Cannot save process preset");
    });
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
            output.write(controller_template(project).toUtf8());
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
            p.dummyToolhead = project.dummyToolhead;
            p.useDummyToolhead = project.useDummyToolhead;
            p.fixtures = project.fixtures;
            p.plateEnabled = project.plateEnabled;
            p.tools = project.tools;
            p.materials = project.materials;
            p.carriageAssembly = project.carriageAssembly;
            p.plate = project.plate;
            p.compute = project.compute;
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
            project.dummyToolhead = p.dummyToolhead;
            project.useDummyToolhead = p.useDummyToolhead;
            project.fixtures = p.fixtures;
            if (p.plateEnabled) {
                if (!project.plateEnabled)
                    enable_plate(project);
                project.tools = p.tools;
                project.carriageAssembly = p.carriageAssembly;
                project.plate = p.plate;
                for (const auto &material : p.materials) {
                    auto found = std::find_if(project.materials.begin(), project.materials.end(),
                                              [&](const MaterialProfile &m) { return m.id == material.id; });
                    if (found == project.materials.end())
                        project.materials.push_back(material);
                    else
                        *found = material;
                }
                project.plate.order.clear();
                for (const auto &object : project.objects)
                    project.plate.order.push_back(object.id);
                synchronize_plate_preview(project);
            }
            invalidate();
            if (!project.path.empty())
                run("Regenerate");
        } catch (const std::exception &e) {
            show_error(e.what());
        }
    });
    file->addAction("Exit", this, &QWidget::close);
    auto processMenu = menuBar()->addMenu("Process");
    processMenu->addAction("Process settings · Orca options", this, &MainWindow::process_settings);
    processMenu->addAction("Import Orca / FiveAxisLab process JSON", this, &MainWindow::import_orca_settings);
    auto guideAction = processMenu->addAction("Import profile centerline guide JSON…", this, [this] {
        if (worker.isRunning())
            return;
        auto file = QFileDialog::getOpenFileName(
            this, "Profile centerline and normals · source model millimetres", {}, "Profile guide (*.json)");
        if (file.isEmpty())
            return;
        try {
            project.profileGuide = import_profile_guide(file, project.partPlacement);
            project.process.profileNormal = true;
            project.invalidate();
            replaying = false;
            refresh();
            log->append("Profile guide loaded. Slice again to generate perpendicular nozzle motion.");
        } catch (const std::exception &e) {
            show_error(e.what());
        }
    });
    guideAction->setObjectName("importProfileGuide");
    auto machineMenu = menuBar()->addMenu("Machine");
    machineMenu->addAction("Setup / toolhead calibration", this, &MainWindow::machine_setup);
    machineMenu->addAction("Bed, fixtures and machine bodies", this, &MainWindow::machine_obstacles);
    auto simulationMenu = menuBar()->addMenu("Simulation");
    simulationMenu->addAction("Try collision avoidance", this, [this] { run("Repair"); });
    simulationMenu->addAction("Validate current motion", this, [this] { run("Validate"); });
    auto axisDemo = [this](Layout machineLayout) {
        if (worker.isRunning())
            return;
        playing = false;
        replaying = false;
        time = 0;
        project = axis_motion_demo(machineLayout);
        fixedFaces.clear();
        forceFaces.clear();
        refresh();
        set_workspace(1);
        seek(0);
        viewport->fit();
        log->append("Tilt / roll demonstration: play or scrub to see coordinated axes. No extrusion; export "
                    "blocked.");
    };
    simulationMenu->addSeparator();
    simulationMenu->addAction("Open rotary-bed axis demonstration", this,
                              [axisDemo] { axisDemo(Layout::RotaryBed); });
    simulationMenu->addAction("Open tilting-head axis demonstration", this,
                              [axisDemo] { axisDemo(Layout::TiltingHead); });
    auto spiralDemo = [this](Layout machineLayout) {
        if (worker.isRunning())
            return;
        playing = false;
        replaying = false;
        time = 0;
        project = spiral_profile_project(machineLayout);
        fixedFaces.clear();
        forceFaces.clear();
        refresh();
        set_workspace(1);
        viewport->fit();
        execute("Generate spiral profile test", [](Project &p, std::atomic_bool *cancel) {
            slice_project(p, cancel);
            p.report = CollisionReport{};
            p.diagnostics << "Spiral five-axis test: profile-normal motion generated; collision validation "
                             "remains required.";
        });
    };
    simulationMenu->addAction("Open spiral five-axis test · rotary bed", this,
                              [spiralDemo] { spiralDemo(Layout::RotaryBed); });
    simulationMenu->addAction("Open spiral five-axis test · tilting head", this,
                              [spiralDemo] { spiralDemo(Layout::TiltingHead); });
    auto xyzDemo = simulationMenu->addAction("Open 3-axis non-planar test", this, [this] {
        if (worker.isRunning())
            return;
        playing = false;
        replaying = false;
        time = 0;
        project = nonplanar_cartesian_project();
        fixedFaces.clear();
        forceFaces.clear();
        refresh();
        set_workspace(1);
        viewport->fit();
        execute("Generate XYZ non-planar test", [](Project &p, std::atomic_bool *cancel) {
            slice_project(p, cancel);
            p.report = CollisionReport{};
            p.diagnostics << "Standard XYZ non-planar paths generated. Validate collisions before export.";
        });
    });
    xyzDemo->setObjectName("openCartesianDemo");
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
        bool reslice =
            project.machine.layout == Layout::Cartesian3Axis || index == int(Layout::Cartesian3Axis);
        project.machine.layout = Layout(index);
        project.process.nonPlanar3Axis = !project.machine.rotary_axes();
        if (!project.machine.rotary_axes())
            project.process.profileNormal = false;
        else {
            project.machine.minB = index ? -75 * pi / 180 : -pi;
            project.machine.maxB = -project.machine.minB;
            if (project.machine.controller == Controller::GenericMarlin)
                project.machine.controller = Controller::Duet;
        }
        if (reslice)
            project.path.clear();
        invalidate();
        if (!project.path.empty())
            run("Regenerate");
    });
    connect(extension, &QCheckBox::toggled, this, [this](bool v) {
        project.machine.klipperExtension = v;
        invalidate();
    });
    connect(processEditor, &ProcessEditor::edited, this, [this] {
        if (auto preset = findChild<QComboBox *>("processPreset")) {
            QSignalBlocker guard(preset);
            preset->setCurrentIndex(0);
        }
        if (!processEditor->valid()) {
            project.invalidate();
            processError->setText(processEditor->validation_error());
            sliceButton->setEnabled(false);
            exportButton->setEnabled(false);
            return;
        }
        auto next = processEditor->settings();
        if (next.nozzleDiameter != project.process.nozzleDiameter)
            project.dummyToolhead = make_dummy_toolhead(next.nozzleDiameter);
        project.process = next;
        project.invalidate();
        replaying = false;
        refresh();
    });
    auto filamentChanged = [this] {
        project.filamentName = filamentName->text();
        project.filamentDensity = filamentDensity->value();
        project.filamentCost = filamentCost->value();
        if (project.plateEnabled && !project.materials.empty()) {
            project.materials[0].name = project.filamentName;
            project.materials[0].density = project.filamentDensity;
            project.materials[0].cost = project.filamentCost;
        }
        refresh_preview(true);
    };
    connect(filamentName, &QLineEdit::editingFinished, this, filamentChanged);
    connect(filamentDensity, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [filamentChanged] { filamentChanged(); });
    connect(filamentCost, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [filamentChanged] { filamentChanged(); });
    connect(dummyToolhead, &QCheckBox::toggled, this, [this](bool enabled) {
        if (enabled && project.dummyToolhead.components.empty())
            project.dummyToolhead = make_dummy_toolhead(project.process.nozzleDiameter);
        project.useDummyToolhead = enabled;
        invalidate();
        if (!project.path.empty())
            run("Regenerate");
    });
    connect(viewport, &Viewport::face_picked, this, [this](int face) {
        if (painting->isChecked()) {
            paint_face(face);
            return;
        }
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
            axisPreview->set_project(&project, simulation_at(project, 0, false).joint);
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
                axisPreview->set_project(&replay, simulation_at(replay, 0, false).joint);
                refresh_preview();
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
    set_workspace(0);
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
    auto currentMaterial = paintMaterial->currentData();
    paintMaterial->clear();
    for (const auto &material : project.materials)
        paintMaterial->addItem(material.name, material.id);
    int index = paintMaterial->findData(currentMaterial);
    if (index >= 0)
        paintMaterial->setCurrentIndex(index);
    painting->setEnabled(project.plateEnabled);
    paintMaterial->setEnabled(project.plateEnabled);
    motionValidated = project.report.exportable() && project.report.fingerprint == project.fingerprint();
    QSignalBlocker a(controller), b(layout), c(extension), j(dummyToolhead), densityGuard(filamentDensity),
        costGuard(filamentCost);
    controller->setCurrentIndex(int(project.machine.controller));
    layout->setCurrentIndex(int(project.machine.layout));
    if (auto model = qobject_cast<QStandardItemModel *>(controller->model()))
        model->item(2)->setEnabled(!project.machine.rotary_axes());
    extension->setChecked(project.machine.klipperExtension);
    extension->setEnabled(project.machine.controller == Controller::Klipper && project.machine.rotary_axes());
    if (auto action = findChild<QAction *>("importProfileGuide"))
        action->setEnabled(project.machine.rotary_axes());
    QSignalBlocker modeGuard(mode);
    mode->setCurrentIndex(replaying ? 1 : 0);
    if (processEditor->valid())
        processEditor->set_settings(project.process);
    processEditor->set_layout(project.machine.layout);
    if (auto axes = findChild<QCheckBox *>("showRotaryAxes"))
        axes->setEnabled(project.machine.rotary_axes());
    processError->setText(processEditor->validation_error());
    sliceButton->setEnabled(processEditor->valid());
    filamentName->setText(project.filamentName);
    filamentDensity->setValue(project.filamentDensity);
    filamentCost->setValue(project.filamentCost);
    partName->setText(project.part.empty() ? "No printable part"
                                           : QFileInfo(project.part.name).fileName() + " · Global settings");
    dummyToolhead->setChecked(project.useDummyToolhead);
    viewport->set_project(&project);
    axisPreview->set_project(&project, simulation_at(project, time, false).joint);
    viewport->set_selected(fixedFaces, forceFaces);
    if (project.plateEnabled) {
        dummyToolhead->setEnabled(false);
        if (auto diameter = processEditor->findChild<QDoubleSpinBox *>("nozzleDiameter")) {
            diameter->setEnabled(false);
            diameter->setToolTip("Installed nozzle diameter is configured per physical tool in Plate/tools");
        }
    } else {
        dummyToolhead->setEnabled(true);
        if (auto diameter = processEditor->findChild<QDoubleSpinBox *>("nozzleDiameter"))
            diameter->setEnabled(true);
    }
    auto events = static_cast<EventTimeline *>(timeline);
    events->marks.clear();
    events->changes.clear();
    if (!project.path.empty() && project.path.back().time > 0)
        for (const auto &event : project.events)
            if (event.kind == EventKind::Attach || event.kind == EventKind::Detach ||
                event.kind == EventKind::Material)
                events->changes.push_back(event.time / project.path.back().time);
    if (!project.path.empty() && project.path.back().time > 0)
        for (const auto &event : project.report.events)
            events->marks.push_back(event.time / project.path.back().time);
    events->update();
    summary->setText(QString("%1 toolhead components\n%2 movements · %3 s\nFEM: %4")
                         .arg(project.active_toolhead().components.size())
                         .arg(project.path.size())
                         .arg(project.path.empty() ? 0 : project.path.back().time, 0, 'f', 1)
                         .arg(project.stress.directional      ? "Complete stress tensors"
                              : project.stress.scalar.empty() ? "No results"
                                                              : "Scalar visualization only"));
    bool current = motionValidated;
    validation->setText(
        current
            ? QString("Motion validated\nMinimum clearance: %1 mm\n%2 checked intervals")
                  .arg(project.report.minimumClearance, 0, 'f', 3)
                  .arg(project.report.checkedIntervals)
            : QString("Export blocked\n%1 collision / unresolved events").arg(project.report.events.size()));
    validation->setStyleSheet(current ? "color:#59d5b7;" : "color:#eeb862;");
    exportButton->setEnabled(current && processEditor->valid());
    refresh_preview();
    double supportVolume = 0;
    for (const auto &sample : project.path)
        if (sample.role == Role::Support && sample.deltaE > 0)
            supportVolume += sample.deltaE * pi * std::pow(project.machine.filamentDiameter / 2, 2);
    summary->setText(summary->text() + QString("\nSupport material: %1 mm³").arg(supportVolume, 0, 'f', 1));
    if (project.useDummyToolhead)
        summary->setText(summary->text() + "\nDummy geometry only");
}
void MainWindow::invalidate() {
    bool valid = project.depositionValid;
    project.invalidate();
    project.depositionValid = valid;
    replaying = false;
    refresh();
}
void MainWindow::execute(const QString &label, std::function<void(Project &, std::atomic_bool *)> task) {
    axisPreview->follow_motion(simulation_at(project, time, false).joint);
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
    if (!processEditor->valid()) {
        show_error(processEditor->validation_error());
        return;
    }
    execute(operation, [operation](Project &p, std::atomic_bool *cancel) {
        if (operation == "Slice") {
            slice_project(p, cancel);
            p.report = validate_motion(p, cancel);
            if (p.report.status == Status::Collision && !(cancel && cancel->load()))
                repair_motion(p, cancel);
        } else if (operation == "Repair") {
            repair_motion(p, cancel);
        } else if (operation == "Regenerate") {
            if (p.plateEnabled) {
                slice_plate(p, cancel);
                p.report = validate_motion(p, cancel);
                return;
            }
            regenerate_joints(p);
            check_deposition(p);
            p.report = validate_motion(p, cancel);
        } else if (operation == "Compare") {
            if (p.plateEnabled)
                throw std::runtime_error("Use each object process and material FEA result; a plate-wide "
                                         "compliance is not meaningful");
            p.comparison = compare_strength(p);
            p.diagnostics
                << QString("Dense-material surrogate compliance: planar %1, curved %2 N·mm; stress alignment "
                           "%3%. Sparse infill and physical failure are outside this constitutive model.")
                       .arg(p.comparison.planarCompliance)
                       .arg(p.comparison.curvedCompliance)
                       .arg(p.comparison.alignment * 100, 0, 'f', 1);
        } else if (operation == "FEA") {
            if (p.plateEnabled) {
                bool any = false;
                for (size_t i = 0; i < p.objects.size(); ++i)
                    if (!p.objects[i].loads.fixedNodes.empty() && !p.objects[i].loads.nodalForces.empty()) {
                        auto result = solve_plate_fea(p, i, cancel);
                        p.diagnostics << QString("%1 FEA compliance %2 N·mm · perfect bond")
                                             .arg(p.objects[i].name)
                                             .arg(result.compliance);
                        any = true;
                    }
                if (!any)
                    throw std::runtime_error(
                        "Select fixed/load faces on one object and configure its loads first");
                synchronize_plate_preview(p);
                return;
            }
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
    ToolheadAssembly cad;
    bool isStep = file.endsWith(".step", Qt::CaseInsensitive) || file.endsWith(".stp", Qt::CaseInsensitive);
    if (isStep && !import_step_dialog(this, file, cad, false))
        return;
    execute("Import part", [file, cad, isStep](Project &p, std::atomic_bool *) {
        if (isStep) {
            p.part = {};
            p.part.name = file;
            for (const auto &component : cad.components) {
                int offset = int(p.part.vertices.size());
                p.part.vertices.insert(p.part.vertices.end(), component.mesh.vertices.begin(),
                                       component.mesh.vertices.end());
                for (auto face : component.mesh.triangles)
                    p.part.triangles.push_back(face + Eigen::Vector3i::Constant(offset));
                p.part.error = std::max(p.part.error, component.mesh.error);
            }
        } else
            p.part = load_mesh(file);
        if (p.plateEnabled) {
            PlateObject object;
            object.id = "object-" + QString::number(p.objects.size() + 1);
            while (std::any_of(p.objects.begin(), p.objects.end(),
                               [&](const auto &o) { return o.id == object.id; }))
                object.id += "x";
            object.mesh = std::move(p.part);
            object.name = object.mesh.name;
            object.materialId = p.materials.front().id;
            object.process = p.process;
            auto bounds = object.mesh.bounds();
            object.placement.translation() =
                Vec3(-bounds.center().x(), -bounds.center().y(), -bounds.min().z()) +
                preferred_plate_position(p.machine);
            p.objects.push_back(object);
            p.plate.order.push_back(object.id);
            p.path.clear();
            p.events.clear();
            p.invalidate();
            synchronize_plate_preview(p);
            return;
        }
        Vec3 offset = p.part.bounds().center();
        offset.z() = p.part.bounds().min().z();
        offset.head<2>() -= preferred_plate_position(p.machine).head<2>();
        for (auto &v : p.part.vertices)
            v -= offset;
        p.partPlacement = translation(-offset);
        p.profileGuide.clear();
        p.process.profileNormal = false;
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
    if (project.plateEnabled) {
        plate_settings();
        return;
    }
    auto file = QFileDialog::getOpenFileName(this, "Import complete toolhead assembly", {},
                                             "STEP assemblies (*.step *.stp)");
    if (file.isEmpty())
        return;
    ToolheadAssembly assembly;
    if (!import_step_dialog(this, file, assembly, true))
        return;
    project.toolhead = std::move(assembly);
    project.useDummyToolhead = false;
    project.invalidate();
    replaying = false;
    refresh();
    log->append("Toolhead imported. Open Machine setup to calibrate the nozzle tip, axis and moving links.");
}
void MainWindow::plate_settings() {
    if (worker.isRunning()) {
        show_error("Wait for or cancel the current job before editing the plate");
        return;
    }
    if (edit_plate(project, this)) {
        paintHistory.clear();
        refresh();
        viewport->fit();
    }
}
void MainWindow::paint_face(int face) {
    if (!project.plateEnabled || paintMaterial->currentData().toString().isEmpty())
        return;
    size_t offset = 0;
    for (auto &object : project.objects) {
        if (face >= int(offset) && face < int(offset + object.mesh.triangles.size())) {
            std::vector<std::vector<MaterialAssignment>> snapshot;
            for (auto &o : project.objects)
                snapshot.push_back(o.assignments);
            paintHistory.push_back(std::move(snapshot));
            if (paintHistory.size() > 32)
                paintHistory.erase(paintHistory.begin());
            int local = face - int(offset);
            auto triangle = object.mesh.triangles[local];
            Vec3 center = (object.mesh.vertices[triangle[0]] + object.mesh.vertices[triangle[1]] +
                           object.mesh.vertices[triangle[2]]) /
                          3;
            MaterialAssignment assignment;
            assignment.id = "paint-" + QString::number(object.assignments.size() + 1);
            assignment.kind = AssignmentKind::Paint;
            assignment.materialId = paintMaterial->currentData().toString();
            if (paintMode->currentIndex() == 0) {
                for (size_t i = 0; i < object.mesh.triangles.size(); ++i) {
                    auto t = object.mesh.triangles[i];
                    Vec3 c = (object.mesh.vertices[t[0]] + object.mesh.vertices[t[1]] +
                              object.mesh.vertices[t[2]]) /
                             3;
                    if ((c - center).norm() <= paintRadius->value() || i == size_t(local))
                        assignment.faces.push_back(int(i));
                }
            } else {
                std::set<int> visited{local};
                std::vector<int> queue{local};
                auto normal = [&](int i) {
                    auto t = object.mesh.triangles[i];
                    return (object.mesh.vertices[t[1]] - object.mesh.vertices[t[0]])
                        .cross(object.mesh.vertices[t[2]] - object.mesh.vertices[t[0]])
                        .normalized()
                        .eval();
                };
                auto n = normal(local);
                for (size_t k = 0; k < queue.size(); ++k)
                    for (size_t i = 0; i < object.mesh.triangles.size(); ++i) {
                        if (visited.count(int(i)) || normal(int(i)).dot(n) < .866)
                            continue;
                        auto a = object.mesh.triangles[queue[k]], b = object.mesh.triangles[i];
                        int shared = 0;
                        for (int x = 0; x < 3; ++x)
                            for (int y = 0; y < 3; ++y)
                                shared += a[x] == b[y];
                        if (shared >= 2) {
                            visited.insert(int(i));
                            queue.push_back(int(i));
                        }
                    }
                assignment.faces = queue;
            }
            object.assignments.push_back(std::move(assignment));
            invalidate();
            refresh();
            return;
        }
        offset += object.mesh.triangles.size();
    }
}
void MainWindow::machine_setup() {
    if (worker.isRunning())
        return;
    if (edit_machine(project, this)) {
        invalidate();
        if (!project.path.empty())
            run("Slice");
    }
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
        auto targetObject = new QComboBox;
        for (const auto &o : project.objects)
            targetObject->addItem(o.name, o.id);
        if (project.plateEnabled)
            layout->addRow("Plate object · results in object coordinates", targetObject);
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
        auto offset = new QLineEdit(vector_text(
                 project.plateEnabled ? Vec3::Zero() : Vec3(project.partPlacement.translation()))),
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
        execute("Import Fusion FEM", [files, options, target = targetObject->currentData().toString()](
                                         Project &p, std::atomic_bool *) {
            if (p.plateEnabled) {
                auto object = std::find_if(p.objects.begin(), p.objects.end(),
                                           [&](const auto &o) { return o.id == target; });
                if (object == p.objects.end())
                    throw std::runtime_error("Missing FEM target object");
                object->stress = import_vtu(files, options);
                object->structuralFingerprint = structural_signature(p, object - p.objects.begin());
                p.invalidate();
                synchronize_plate_preview(p);
                return;
            }
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
    if (project.plateEnabled) {
        int offset = 0, index = -1;
        for (size_t i = 0; i < project.objects.size(); ++i) {
            int end = offset + project.objects[i].mesh.triangles.size();
            auto belongs = [&](int face) { return face >= offset && face < end; };
            if (!fixedFaces.empty() && !forceFaces.empty() &&
                std::all_of(fixedFaces.begin(), fixedFaces.end(), belongs) &&
                std::all_of(forceFaces.begin(), forceFaces.end(), belongs)) {
                index = int(i);
                break;
            }
            offset = end;
        }
        if (index < 0) {
            show_error("Select fixed and loaded faces belonging to the same plate object");
            return;
        }
        auto plate = project;
        auto fixed = fixedFaces, force = forceFaces;
        auto &o = plate.objects[index];
        Project local;
        local.part = o.mesh;
        local.volume = o.volume;
        local.loads = o.loads;
        project = local;
        fixedFaces.clear();
        forceFaces.clear();
        for (int f : fixed)
            fixedFaces.insert(f - offset);
        for (int f : force)
            forceFaces.insert(f - offset);
        setup_loads();
        o.volume = project.volume;
        o.loads = project.loads;
        o.stress = {};
        o.structuralFingerprint.clear();
        if (auto material = std::find_if(plate.materials.begin(), plate.materials.end(),
                                         [&](const auto &m) { return m.id == o.materialId; });
            material != plate.materials.end())
            material->mechanical = o.loads.material;
        project = std::move(plate);
        fixedFaces = std::move(fixed);
        forceFaces = std::move(force);
        project.invalidate();
        synchronize_plate_preview(project);
        refresh();
        return;
    }
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
    if (axisPreview->overriding()) {
        show_error("Return to Follow toolpath before exporting validated motion");
        return;
    }
    if (!processEditor->valid()) {
        show_error(processEditor->validation_error());
        return;
    }
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
        processEditor->set_settings(project.process);
        refresh();
        set_workspace(project.path.empty() ? 0 : 1);
        viewport->fit();
        seek(0);
    } catch (const std::exception &e) {
        show_error(e.what());
    }
}
void MainWindow::toggle_play() {
    if (axisPreview->overriding()) {
        axisPreview->toggle_animation();
        return;
    }
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
    axisPreview->follow_motion(simulation_at(p, time, false).joint);
    update_move_preview();
}
void MainWindow::capture(const QString &filename) {
    bool posePreview = axisPreview->overriding();
    set_workspace(posePreview || !project.path.empty() ? 1 : 0);
    if (!posePreview)
        seek(project.path.empty() ? 0 : project.path.back().time * .65);
    if (posePreview)
        axisPreview->pause();
    viewport->fit();
    viewport->repaint();
    grab().save(filename);
}
