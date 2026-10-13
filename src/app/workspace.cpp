#include "core/controllers.h"
#include "core/plate.h"
#include "main_window.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTextCharFormat>
#include <QTextCursor>
#include <QVBoxLayout>
#include <array>
using namespace fiveaxis;
static QString duration_text(double seconds) {
    int n = int(std::round(seconds));
    return n >= 3600 ? QString("%1h %2m").arg(n / 3600).arg(n % 3600 / 60)
                     : QString("%1m %2s").arg(n / 60).arg(n % 60);
}
static QString short_time(double seconds) {
    int n = int(std::round(seconds));
    return n >= 3600 ? QString::number(n / 3600.0, 'f', 1) + "h"
           : n >= 60 ? QString::number(n / 60.0, 'f', 1) + "m"
                     : QString::number(n) + "s";
}
QWidget *MainWindow::create_preview_inspector() {
    auto panel = new QWidget;
    panel->setObjectName("previewInspector");
    panel->setFixedWidth(300);
    auto container = new QVBoxLayout(panel);
    container->setContentsMargins(0, 0, 0, 0);
    auto scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFrameShape(QFrame::NoFrame);
    auto contents = new QWidget;
    scroll->setWidget(contents);
    container->addWidget(scroll);
    auto outer = new QVBoxLayout(contents);
    outer->setContentsMargins(4, 4, 4, 4);
    outer->setSpacing(5);
    auto heading = new QLabel("Slicing result · line type");
    heading->setStyleSheet("font-weight:600;");
    outer->addWidget(heading);
    axisPreview = new AxisPreview;
    outer->addWidget(axisPreview);
    connect(axisPreview, &AxisPreview::pose_changed, this, &MainWindow::update_axis_pose);
    roleStatistics = new QTableWidget(role_count, 5);
    roleStatistics->setObjectName("roleStatistics");
    roleStatistics->setHorizontalHeaderLabels({"Line type", "Time", "%", "m", "g"});
    roleStatistics->verticalHeader()->hide();
    roleStatistics->setShowGrid(false);
    roleStatistics->setEditTriggers(QAbstractItemView::NoEditTriggers);
    roleStatistics->setSelectionMode(QAbstractItemView::NoSelection);
    roleStatistics->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    roleStatistics->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    roleStatistics->setColumnWidth(0, 112);
    roleStatistics->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
    roleStatistics->setColumnWidth(1, 50);
    roleStatistics->setMinimumHeight(110);
    roleStatistics->setMaximumHeight(240);
    outer->addWidget(roleStatistics);
    for (int r = 0; r < role_count; ++r) {
        auto item = new QTableWidgetItem(role_name(Role(r)));
        item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
        item->setForeground(toolpath_color(Role(r)));
        roleStatistics->setItem(r, 0, item);
        roleStatistics->setRowHeight(r, 23);
        for (int col = 1; col < 5; ++col)
            roleStatistics->setItem(r, col, new QTableWidgetItem("0"));
    }
    connect(roleStatistics, &QTableWidget::itemChanged, this, [this](QTableWidgetItem *item) {
        if (!updatingPreview && item->column() == 0)
            viewport->set_role_visible(Role(item->row()), item->checkState() == Qt::Checked);
    });
    totals = new QLabel;
    totals->setObjectName("filamentEstimates");
    totals->setWordWrap(true);
    outer->addWidget(totals);
    auto range = new QGroupBox("Layer range");
    auto rangeRow = new QHBoxLayout(range);
    rangeRow->setContentsMargins(5, 12, 5, 5);
    lowerLayer = new QSlider(Qt::Vertical);
    lowerLayer->setObjectName("lowerLayer");
    upperLayer = new QSlider(Qt::Vertical);
    upperLayer->setObjectName("upperLayer");
    lowerLayer->setMinimumHeight(55);
    upperLayer->setMinimumHeight(55);
    layerLabel = new QLabel;
    layerLabel->setWordWrap(true);
    rangeRow->addWidget(layerLabel, 1);
    rangeRow->addWidget(lowerLayer);
    rangeRow->addWidget(upperLayer);
    outer->addWidget(range);
    auto updateRange = [this](bool upper) {
        if (updatingPreview)
            return;
        QSignalBlocker low(lowerLayer), high(upperLayer);
        if (lowerLayer->value() > upperLayer->value()) {
            if (upper)
                lowerLayer->setValue(upperLayer->value());
            else
                upperLayer->setValue(lowerLayer->value());
        }
        int lo = lowerLayer->value(), hi = upperLayer->value();
        viewport->set_layer_range(lo, hi);
        layerLabel->setText(QString("Layers %1 – %2\n(%3 total)\nGeometry filters only")
                                .arg(lo + 1)
                                .arg(hi + 1)
                                .arg(upperLayer->maximum() + 1));
        auto &p = replaying ? replay : project;
        double end = 0;
        for (const auto &move : p.path)
            if (move.layer == hi)
                end = std::max(end, move.time);
        playing = false;
        seek(end);
    };
    connect(lowerLayer, &QSlider::valueChanged, this, [updateRange] { updateRange(false); });
    connect(upperLayer, &QSlider::valueChanged, this, [updateRange] { updateRange(true); });
    auto label = new QLabel("Current movement");
    label->setStyleSheet("font-weight:600;");
    outer->addWidget(label);
    movePreview = new QTextEdit;
    movePreview->setObjectName("moveCodePreview");
    movePreview->setReadOnly(true);
    movePreview->setMinimumHeight(80);
    movePreview->setStyleSheet("font-family:Consolas;font-size:10px;background:#394345;color:#e9eded;");
    outer->addWidget(movePreview, 1);
    return panel;
}
void MainWindow::refresh_preview(bool preserveRange) {
    updatingPreview = true;
    auto &p = replaying ? replay : project;
    std::array<double, role_count> times{}, extrusion{}, mass{};
    std::map<QString, double> used, waste;
    double materialCost = 0;
    int maximum = 0;
    for (size_t i = 0; i < p.path.size(); ++i) {
        const auto &s = p.path[i];
        int role = int(s.role);
        maximum = std::max(maximum, s.layer);
        if (role >= 0 && role < role_count) {
            times[role] += i ? std::max(0.0, s.time - p.path[i - 1].time) : 0;
            double length =
                s.event == EventKind::Load || s.event == EventKind::Unload ? 0 : std::max(0.0, s.deltaE);
            extrusion[role] += length;
            const auto *material = p.plateEnabled ? find_material(p, s.materialId) : nullptr;
            double diameter = material ? material->diameter : p.machine.filamentDiameter,
                   density = material ? material->density : p.filamentDensity,
                   cost = material ? material->cost : p.filamentCost;
            double grams = length * pi * std::pow(diameter / 2, 2) / 1000 * density;
            mass[role] += grams;
            materialCost += grams / 1000 * cost;
            used[s.materialId] += length;
            if (s.role == Role::Purge)
                waste[s.materialId] += length;
        }
    }
    double total = p.path.empty() ? 0 : p.path.back().time, filament = 0,
           area = pi * std::pow(p.machine.filamentDiameter / 2, 2);
    for (int r = 0; r < role_count; ++r) {
        filament += extrusion[r];
        auto write = [&](int c, const QString &text) { roleStatistics->item(r, c)->setText(text); };
        write(1, short_time(times[r]));
        write(2, QString::number(total > 0 ? times[r] / total * 100 : 0, 'f', 1));
        write(3, QString::number(extrusion[r] / 1000, 'f', 2));
        write(4, QString::number(mass[r], 'f', 2));
    }
    double grams = 0;
    for (double amount : mass)
        grams += amount;
    totals->setText(QString("Total filament: %1 m · %2 g\nMaterial cost: %3 EUR\nPlanned time: %4\n%5\n%6")
                        .arg(filament / 1000, 0, 'f', 2)
                        .arg(grams, 0, 'f', 2)
                        .arg(materialCost, 0, 'f', 2)
                        .arg(duration_text(total))
                        .arg(project.filamentName)
                        .arg(project.useDummyToolhead ? "Dummy toolhead · synthetic geometry"
                                                      : "Configured toolhead geometry"));
    if (p.plateEnabled) {
        QString text = QString("Plate total: %1 m, %2 g\nMaterial cost: %3 EUR\nPlanned time: %4")
                           .arg(filament / 1000, 0, 'f', 3)
                           .arg(grams, 0, 'f', 3)
                           .arg(materialCost, 0, 'f', 3)
                           .arg(duration_text(total));
        for (const auto &[id, length] : used) {
            auto m = find_material(p, id);
            if (m)
                text += QString("\n%1: %2 m, purge %3 m")
                            .arg(m->name)
                            .arg(length / 1000, 0, 'f', 3)
                            .arg(waste[id] / 1000, 0, 'f', 3);
        }
        totals->setText(text);
    }
    int lo = preserveRange ? std::min(lowerLayer->value(), maximum) : 0,
        hi = preserveRange ? std::min(upperLayer->value(), maximum) : maximum;
    lowerLayer->setRange(0, maximum);
    upperLayer->setRange(0, maximum);
    lowerLayer->setValue(lo);
    upperLayer->setValue(hi);
    viewport->set_layer_range(lo, hi);
    layerLabel->setText(p.path.empty()
                            ? "No sliced layers"
                            : QString("Layers %1 – %2\nGeometry filters only").arg(lo + 1).arg(hi + 1));
    lastPreviewMove = -1;
    updatingPreview = false;
    update_move_preview();
}
void MainWindow::update_move_preview() {
    auto &p = replaying ? replay : project;
    if (p.path.empty()) {
        movePreview->setPlainText("Slice a part to inspect motion.");
        return;
    }
    auto state = simulation_at(p, time, false);
    if (state.move == lastPreviewMove)
        return;
    lastPreviewMove = state.move;
    movePreview->clear();
    movePreview->append(
        replaying
            ? "CONTROLLER REPLAY · relative E"
            : "PLANNED MOTION · relative E\nCoordinates below are a preview; export requires validation.");
    for (int i = std::max(0, state.move - 3); i < std::min<int>(p.path.size(), state.move + 8); ++i) {
        const auto &s = p.path[i];
        auto axes = controller_axes(p.machine, p.active_toolhead(), s.joint);
        QTextCharFormat format;
        format.setForeground(i == state.move ? QColor("#00eb76") : QColor("#d5dddd"));
        format.setFontWeight(i == state.move ? QFont::Bold : QFont::Normal);
        format.setFontPointSize(9);
        double feed = 0;
        if (i > 0) {
            const auto &a = p.path[i - 1];
            auto previous = controller_axes(p.machine, p.active_toolhead(), a.joint);
            feed = feed_metric(p.machine, previous, axes, s.deltaE) / std::max(.001, s.time - a.time) * 60;
        }
        auto cursor = movePreview->textCursor();
        cursor.movePosition(QTextCursor::End);
        if (!p.machine.rotary_axes()) {
            cursor.insertText(
                QString("\n%1 · L%2 · %3\n%4 X%5 Y%6 Z%7 E%8 F%9\n; fixed nozzle +Z · t=%10 s\n")
                    .arg(i)
                    .arg(s.layer + 1)
                    .arg(role_name(s.role))
                    .arg(i == 0 ? "Initial pose" : "G1")
                    .arg(axes.xyz.x(), 0, 'f', 3)
                    .arg(axes.xyz.y(), 0, 'f', 3)
                    .arg(axes.xyz.z(), 0, 'f', 3)
                    .arg(s.deltaE, 0, 'f', 5)
                    .arg(feed, 0, 'f', 2)
                    .arg(s.time, 0, 'f', 2),
                format);
            continue;
        }
        cursor.insertText(
            QString("\n%1 · L%2 · %3\n%13 X%4 Y%5 Z%6 A%7 %8%9 E%10 F%14\n; t=%11 s · requested %12 mm/s\n")
                .arg(i)
                .arg(s.layer + 1)
                .arg(role_name(s.role))
                .arg(axes.xyz.x(), 0, 'f', 3)
                .arg(axes.xyz.y(), 0, 'f', 3)
                .arg(axes.xyz.z(), 0, 'f', 3)
                .arg(s.joint.a * 180 / pi, 0, 'f', 3)
                .arg(p.machine.axis2())
                .arg(s.joint.b * 180 / pi, 0, 'f', 3)
                .arg(s.deltaE, 0, 'f', 5)
                .arg(s.time, 0, 'f', 2)
                .arg(s.speed, 0, 'f', 1)
                .arg(i == 0 ? "Initial pose" : "G1")
                .arg(feed, 0, 'f', 2),
            format);
    }
    movePreview->moveCursor(QTextCursor::Start);
}
void MainWindow::set_workspace(int index) {
    if (index == 0 && axisPreview->overriding())
        axisPreview->follow_motion(simulation_at(replaying ? replay : project, time, false).joint);
    QSignalBlocker guard(workspaces);
    workspaces->setCurrentIndex(index);
    viewport->set_prepare(index == 0);
    auto inspector = findChild<QWidget *>("previewInspector");
    if (inspector)
        inspector->setVisible(index == 1);
    for (const auto &name : {"simulationControls", "simulationPaths", "simulationTransport"})
        if (auto widget = findChild<QWidget *>(name))
            widget->setVisible(index == 1);
    if (index == 1)
        viewport->set_time(time);
}
void MainWindow::update_axis_pose(const JointState &pose, bool overrideMotion) {
    if (overrideMotion)
        playing = false;
    bool wasOverride = viewport->axis_pose_active();
    viewport->set_axis_pose(pose, overrideMotion);
    bool current = motionValidated && project.report.exportable();
    exportButton->setEnabled(!overrideMotion && current && processEditor->valid());
    if (overrideMotion) {
        validation->setText("Axis pose preview · motion not validated");
        validation->setStyleSheet("color:#ad6a20;");
        movePreview->setPlainText(
            "AXIS POSE PREVIEW\nNo generated G-code for this pose.\n\nThe nozzle follows a fixed point on "
            "the workpiece. No new material is deposited.\n\nChoose Follow toolpath to return to planned or "
            "exported motion.");
    } else {
        validation->setText(current
                                ? QString("Motion validated\nMinimum clearance: %1 mm\n%2 checked intervals")
                                      .arg(project.report.minimumClearance, 0, 'f', 3)
                                      .arg(project.report.checkedIntervals)
                                : QString("Export blocked\n%1 collision / unresolved events")
                                      .arg(project.report.events.size()));
        validation->setStyleSheet(current ? "color:#27845f;" : "color:#ad6a20;");
        if (wasOverride) {
            lastPreviewMove = -1;
            update_move_preview();
        }
    }
}
