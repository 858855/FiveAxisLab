#include "plate_editor.h"
#include "core/compute.h"
#include "core/geometry.h"
#include "core/plate.h"
#include "core/project.h"
#include "process_editor.h"
#include "step_dialog.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
using namespace fiveaxis;
namespace {
QDoubleSpinBox *number(double value, double low = 0, double high = 100000) {
    auto box = new QDoubleSpinBox;
    box->setDecimals(4);
    box->setRange(low, high);
    box->setValue(value);
    return box;
}
QString vector_text(const Vec3 &v) {
    return QString("%1, %2, %3").arg(v.x()).arg(v.y()).arg(v.z());
}
Vec3 read_vector(const QString &text) {
    auto values = text.split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts);
    if (values.size() != 3)
        throw std::runtime_error("Enter three coordinates");
    Vec3 v;
    for (int i = 0; i < 3; ++i) {
        bool ok;
        v[i] = values[i].toDouble(&ok);
        if (!ok || !std::isfinite(v[i]))
            throw std::runtime_error("Invalid coordinate");
    }
    return v;
}
void item(QTableWidget *table, int row, int column, const QString &text) {
    table->setItem(row, column, new QTableWidgetItem(text));
}
QString cell(QTableWidget *table, int row, int column) {
    return table->item(row, column) ? table->item(row, column)->text() : QString{};
}
double value(QTableWidget *table, int row, int column) {
    bool ok;
    double result = cell(table, row, column).toDouble(&ok);
    if (!ok || !std::isfinite(result))
        throw std::runtime_error("Invalid numeric table cell");
    return result;
}
} // namespace
bool edit_plate(Project &project, QWidget *parent) {
    Project copy = project;
    enable_plate(copy);
    QDialog dialog(parent);
    dialog.setObjectName("plateEditor");
    dialog.setWindowTitle("Plate · objects, materials, tools and compute");
    dialog.resize(1120, 800);
    auto outer = new QVBoxLayout(&dialog);
    auto tabs = new QTabWidget;
    outer->addWidget(tabs, 1);
    auto objectsPage = new QWidget;
    auto ol = new QVBoxLayout(objectsPage);
    auto objects = new QTableWidget(0, 7);
    objects->setObjectName("plateObjects");
    objects->setHorizontalHeaderLabels({"ID", "Name", "Material ID", "Tool override (empty = auto)",
                                        "Placement X", "Placement Y", "Placement Z"});
    ol->addWidget(objects, 1);
    auto objectButtons = new QHBoxLayout;
    auto addObject = new QPushButton("Add geometry");
    auto duplicate = new QPushButton("Duplicate");
    auto removeObject = new QPushButton("Remove");
    objectButtons->addWidget(addObject);
    objectButtons->addWidget(duplicate);
    objectButtons->addWidget(removeObject);
    ol->addLayout(objectButtons);
    auto order = new QListWidget;
    order->setObjectName("objectOrder");
    order->setDragDropMode(QAbstractItemView::InternalMove);
    order->setMaximumHeight(130);
    ol->addWidget(new QLabel("Complete-object order · drag rows to reorder"));
    ol->addWidget(order);
    auto sequence = new QComboBox;
    sequence->addItems({"Layer-by-layer", "Complete objects"});
    sequence->setCurrentIndex(int(copy.plate.sequence));
    sequence->setObjectName("printSequence");
    auto automaticOrder = new QCheckBox("Find a validated object order automatically");
    automaticOrder->setChecked(copy.plate.automaticOrder);
    auto automaticTools = new QCheckBox("Select installed tools/nozzles automatically");
    automaticTools->setChecked(copy.plate.automaticTools);
    ol->addWidget(sequence);
    ol->addWidget(automaticOrder);
    ol->addWidget(automaticTools);
    tabs->addTab(objectsPage, "Objects and order");
    auto storeObjects = [&] {
        for (int r = 0; r < objects->rowCount(); ++r) {
            auto &o = copy.objects[r];
            o.name = cell(objects, r, 1);
            o.materialId = cell(objects, r, 2);
            o.toolId = cell(objects, r, 3);
            for (int k = 0; k < 3; ++k)
                o.placement.translation()[k] = value(objects, r, k + 4);
        }
    };
    auto refreshObjects = [&] {
        objects->setRowCount(int(copy.objects.size()));
        order->clear();
        for (size_t i = 0; i < copy.objects.size(); ++i) {
            auto &o = copy.objects[i];
            item(objects, i, 0, o.id);
            objects->item(i, 0)->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            item(objects, i, 1, o.name);
            item(objects, i, 2, o.materialId);
            item(objects, i, 3, o.toolId);
            for (int k = 0; k < 3; ++k)
                item(objects, i, 4 + k, QString::number(o.placement.translation()[k]));
        }
        for (const auto &id : copy.plate.order)
            order->addItem(id);
    };
    refreshObjects();
    QObject::connect(addObject, &QPushButton::clicked, &dialog, [&] {
        auto file = QFileDialog::getOpenFileName(&dialog, "Add printable geometry", {},
                                                 "Geometry (*.stl *.obj *.3mf *.step *.stp)");
        if (file.isEmpty())
            return;
        try {
            storeObjects();
            PlateObject o;
            o.id = "object-" + QString::number(copy.objects.size() + 1);
            while (
                std::any_of(copy.objects.begin(), copy.objects.end(), [&](auto &v) { return v.id == o.id; }))
                o.id += "x";
            o.mesh = load_mesh(file);
            o.name = o.mesh.name;
            o.materialId = copy.materials.front().id;
            o.process = copy.process;
            auto bounds = o.mesh.bounds();
            o.placement.translation() = Vec3(-bounds.center().x(), -bounds.center().y(), -bounds.min().z());
            copy.objects.push_back(o);
            copy.plate.order.push_back(o.id);
            refreshObjects();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Import", e.what());
        }
    });
    QObject::connect(duplicate, &QPushButton::clicked, &dialog, [&] {
        try {
            int row = objects->currentRow();
            if (row < 0)
                return;
            storeObjects();
            auto o = copy.objects[row];
            o.id += "-copy";
            while (
                std::any_of(copy.objects.begin(), copy.objects.end(), [&](auto &v) { return v.id == o.id; }))
                o.id += "x";
            o.name += " copy";
            o.placement.translation().x() += o.mesh.bounds().sizes().x() + 10;
            copy.objects.push_back(o);
            copy.plate.order.push_back(o.id);
            refreshObjects();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    QObject::connect(removeObject, &QPushButton::clicked, &dialog, [&] {
        try {
            int row = objects->currentRow();
            if (row < 0 || copy.objects.size() == 1)
                return;
            storeObjects();
            auto id = copy.objects[row].id;
            copy.objects.erase(copy.objects.begin() + row);
            copy.plate.order.erase(std::remove(copy.plate.order.begin(), copy.plate.order.end(), id),
                                   copy.plate.order.end());
            refreshObjects();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    auto objectProcess = new QPushButton("Edit selected object's process override");
    ol->addWidget(objectProcess);
    QObject::connect(objectProcess, &QPushButton::clicked, &dialog, [&] {
        int r = objects->currentRow();
        if (r < 0)
            return;
        auto &o = copy.objects[r];
        QDialog editor(&dialog);
        editor.setWindowTitle(o.name + " · process");
        auto layout = new QVBoxLayout(&editor);
        auto inherit = new QCheckBox("Inherit plate process settings");
        inherit->setChecked(o.inheritProcess);
        layout->addWidget(inherit);
        auto process = new ProcessEditor;
        process->set_settings(o.inheritProcess ? copy.process : o.process);
        process->set_layout(copy.machine.layout);
        layout->addWidget(process);
        auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        layout->addWidget(buttons);
        QObject::connect(buttons, &QDialogButtonBox::accepted, &editor, [&] {
            if (process->valid()) {
                o.inheritProcess = inherit->isChecked();
                o.process = process->settings();
                editor.accept();
            }
        });
        QObject::connect(buttons, &QDialogButtonBox::rejected, &editor, &QDialog::reject);
        editor.resize(650, 750);
        editor.exec();
    });
    auto materialPage = new QWidget;
    auto ml = new QVBoxLayout(materialPage);
    auto materials = new QTableWidget(0, 17);
    materials->setObjectName("materialProfiles");
    materials->setHorizontalHeaderLabels({"ID", "Name", "Color", "Filament mm", "Print °C", "Standby °C",
                                          "Flow mm³/s", "Density", "EUR/kg", "Min °C", "Max °C", "Bed min",
                                          "Bed max", "Fan 0..1", "Young MPa (blank=unknown)", "Poisson",
                                          "Transverse MPa"});
    ml->addWidget(materials);
    auto addMaterial = new QPushButton("Add material");
    ml->addWidget(addMaterial);
    tabs->addTab(materialPage, "Materials");
    auto refreshMaterials = [&] {
        materials->setRowCount(copy.materials.size());
        for (size_t i = 0; i < copy.materials.size(); ++i) {
            auto &m = copy.materials[i];
            QStringList row{m.id,
                            m.name,
                            m.color,
                            QString::number(m.diameter),
                            QString::number(m.temperature),
                            QString::number(m.standby),
                            QString::number(m.maxFlow),
                            QString::number(m.density),
                            QString::number(m.cost),
                            QString::number(m.minTemperature),
                            QString::number(m.maxTemperature),
                            QString::number(m.bedMin),
                            QString::number(m.bedMax),
                            QString::number(m.fan),
                            m.mechanical ? QString::number(m.mechanical->young) : QString{},
                            m.mechanical ? QString::number(m.mechanical->poisson) : QString{},
                            m.mechanical ? QString::number(m.mechanical->transverseYoung) : QString{}};
            for (int c = 0; c < row.size(); ++c)
                item(materials, i, c, row[c]);
            materials->item(i, 0)->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        }
    };
    refreshMaterials();
    auto storeMaterials = [&] {
        for (int r = 0; r < materials->rowCount(); ++r) {
            auto &m = copy.materials[r];
            m.name = cell(materials, r, 1);
            m.color = cell(materials, r, 2);
            m.diameter = value(materials, r, 3);
            m.temperature = value(materials, r, 4);
            m.standby = value(materials, r, 5);
            m.maxFlow = value(materials, r, 6);
            m.density = value(materials, r, 7);
            m.cost = value(materials, r, 8);
            m.minTemperature = value(materials, r, 9);
            m.maxTemperature = value(materials, r, 10);
            m.bedMin = value(materials, r, 11);
            m.bedMax = value(materials, r, 12);
            m.fan = value(materials, r, 13);
            if (cell(materials, r, 14).isEmpty())
                m.mechanical.reset();
            else
                m.mechanical =
                    Material{value(materials, r, 14), value(materials, r, 15), value(materials, r, 16)};
        }
    };
    QObject::connect(addMaterial, &QPushButton::clicked, &dialog, [&] {
        try {
            storeMaterials();
            MaterialProfile m;
            m.id = "material-" + QString::number(copy.materials.size() + 1);
            m.name = "New material";
            copy.materials.push_back(m);
            refreshMaterials();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    auto toolPage = new QWidget;
    auto tl = new QHBoxLayout(toolPage);
    auto toolList = new QListWidget;
    toolList->setObjectName("physicalTools");
    toolList->setMaximumWidth(210);
    auto listColumn = new QVBoxLayout;
    listColumn->addWidget(toolList);
    auto addTool = new QPushButton("Add physical tool");
    listColumn->addWidget(addTool);
    tl->addLayout(listColumn);
    auto toolRight = new QWidget;
    auto toolLayout = new QVBoxLayout(toolRight);
    auto form = new QFormLayout;
    toolLayout->addLayout(form);
    auto toolScroll = new QScrollArea;
    toolScroll->setWidgetResizable(true);
    toolScroll->setWidget(toolRight);
    tl->addWidget(toolScroll, 1);
    auto toolName = new QLineEdit, extruder = new QLineEdit, heater = new QLineEdit;
    auto drive = new QSpinBox, heaterIndex = new QSpinBox;
    drive->setRange(0, 255);
    heaterIndex->setRange(0, 255);
    auto toolNumber = new QSpinBox;
    toolNumber->setRange(0, 255);
    auto dummy = new QCheckBox("Use dimensioned dummy toolhead");
    auto detachable = new QCheckBox("Detachable toolhead");
    auto calibrated = new QCheckBox("Installed nozzle and CAD/TCP calibration confirmed");
    auto automaticLimits = new QCheckBox("Auto-fill bead limits when nozzle diameter changes");
    automaticLimits->setObjectName("autoNozzleBeadLimits");
    automaticLimits->setToolTip(
        "Minimum height and width = half the nozzle diameter; maximum = twice the diameter. "
        "All limits remain editable. Uncheck to keep custom limits when changing diameter.");
    auto diameter = number(.4, .05, 5), minHeight = number(.05, .001, 10), maxHeight = number(.3, .001, 10),
         minWidth = number(.32, .01, 10), maxWidth = number(.8, .01, 10), maxFlow = number(10, .01, 500);
    diameter->setObjectName("toolNozzleDiameter");
    minHeight->setObjectName("toolMinBeadHeight");
    maxHeight->setObjectName("toolMaxBeadHeight");
    minWidth->setObjectName("toolMinBeadWidth");
    maxWidth->setObjectName("toolMaxBeadWidth");
    calibrated->setObjectName("toolNozzleCalibrated");
    auto tcp = new QLineEdit, axis = new QLineEdit, mount = new QLineEdit, dock = new QLineEdit;
    form->addRow("Tool name", toolName);
    form->addRow("Controller tool number", toolNumber);
    form->addRow("Duet extruder drive", drive);
    form->addRow("Duet heater index", heaterIndex);
    form->addRow("Extruder", extruder);
    form->addRow("Heater", heater);
    form->addRow(dummy);
    form->addRow(detachable);
    form->addRow("Installed nozzle diameter · mm", diameter);
    form->addRow(automaticLimits);
    form->addRow("Minimum local bead height", minHeight);
    form->addRow("Maximum local bead height", maxHeight);
    form->addRow("Minimum bead width", minWidth);
    form->addRow("Maximum bead width", maxWidth);
    form->addRow("Maximum nozzle flow", maxFlow);
    form->addRow("CAD nozzle TCP · x,y,z", tcp);
    form->addRow("CAD nozzle axis", axis);
    form->addRow("Mount translation · x,y,z", mount);
    form->addRow("Parked CAD origin · x,y,z", dock);
    form->addRow(calibrated);
    auto toolButtons = new QHBoxLayout;
    auto importStep = new QPushButton("Import toolhead STEP");
    auto importNozzle = new QPushButton("Import separate nozzle STEP");
    auto applyTool = new QPushButton("Apply tool");
    toolButtons->addWidget(importStep);
    toolButtons->addWidget(importNozzle);
    toolButtons->addWidget(applyTool);
    toolLayout->addLayout(toolButtons);
    auto details = new QPlainTextEdit;
    details->setObjectName("toolRecipeJson");
    details->setMaximumHeight(200);
    toolLayout->addWidget(
        new QLabel("Routes and calibrated pickup/release recipe · JSON (carriage XYZ mm, angles radians)"));
    toolLayout->addWidget(details);
    auto nozzleMode = new QComboBox;
    nozzleMode->addItems({"Component in toolhead CAD", "Separate nozzle STEP", "Parameterized dummy nozzle"});
    auto nozzleMount = new QLineEdit;
    auto nozzleRotation = new QLineEdit, separateTip = new QLineEdit, separateAxis = new QLineEdit;
    auto contactRadius = number(.6, 0, 100), contactHeight = number(1, 0, 100);
    form->addRow("Nozzle mount XYZ rotation · degrees", nozzleRotation);
    form->addRow("Separate nozzle source CAD tip", separateTip);
    form->addRow("Separate nozzle source CAD axis", separateAxis);
    form->addRow("Deposition contact radius · mm", contactRadius);
    form->addRow("Deposition contact height · mm", contactHeight);
    form->addRow("Nozzle geometry source", nozzleMode);
    form->addRow("Separate/dummy nozzle mount XYZ", nozzleMount);
    auto components = new QTableWidget(0, 5);
    components->setObjectName("toolComponents");
    components->setHorizontalHeaderLabels({"Component ID", "Name", "Moving link", "Nozzle", "Visual only"});
    components->setMinimumHeight(180);
    toolLayout->addWidget(new QLabel("Physical components · all included unless explicitly visual only"));
    toolLayout->addWidget(components);
    int selected = -1;
    auto loadTool = [&](int index) {
        if (index < 0 || index >= int(copy.tools.size()))
            return;
        selected = index;
        QSignalBlocker diameterGuard(diameter), dummyGuard(dummy), automaticLimitsGuard(automaticLimits);
        auto &t = copy.tools[index];
        auto a = t.geometry();
        toolName->setText(t.name);
        toolNumber->setValue(t.controllerTool);
        drive->setValue(t.duetDrive);
        heaterIndex->setValue(t.duetHeater);
        extruder->setText(t.extruder);
        heater->setText(t.heater);
        dummy->setChecked(t.useDummy);
        detachable->setChecked(t.detachable);
        calibrated->setChecked(t.nozzle.calibrated);
        diameter->setValue(t.nozzle.diameter);
        automaticLimits->setChecked(t.nozzle.autoBeadLimits);
        minHeight->setValue(t.nozzle.minHeight);
        maxHeight->setValue(t.nozzle.maxHeight);
        minWidth->setValue(t.nozzle.minWidth);
        maxWidth->setValue(t.nozzle.maxWidth);
        maxFlow->setValue(t.nozzle.maxFlow);
        tcp->setText(vector_text(a.tcp));
        axis->setText(vector_text(a.axis));
        mount->setText(vector_text(a.mount.translation()));
        dock->setText(vector_text(t.dock.translation()));
        nozzleMode->setCurrentIndex(t.nozzle.dummyNozzle                          ? 2
                                    : !t.nozzle.separateNozzle.components.empty() ? 1
                                                                                  : 0);
        nozzleMount->setText(vector_text(t.nozzle.nozzleMount.translation()));
        auto angles = t.nozzle.nozzleMount.linear().eulerAngles(2, 1, 0);
        nozzleRotation->setText(vector_text(Vec3(angles.z(), angles.y(), angles.x()) * 180 / pi));
        separateTip->setText(vector_text(t.nozzle.separateNozzle.tcp));
        separateAxis->setText(vector_text(t.nozzle.separateNozzle.axis));
        contactRadius->setValue(a.contactRadius);
        contactHeight->setValue(a.contactHeight);
        components->setRowCount(a.components.size());
        for (size_t i = 0; i < a.components.size(); ++i) {
            const auto &c = a.components[i];
            item(components, i, 0, c.id);
            components->item(i, 0)->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            item(components, i, 1, c.name);
            auto link = new QComboBox;
            link->addItems({"Carriage", "Rotary A stage", "Final nozzle link"});
            link->setCurrentIndex(int(c.link));
            components->setCellWidget(i, 2, link);
            auto nozzle = new QCheckBox;
            nozzle->setChecked(c.nozzle);
            components->setCellWidget(i, 3, nozzle);
            auto visual = new QCheckBox;
            visual->setChecked(c.visualOnly);
            components->setCellWidget(i, 4, visual);
        }
        auto json = plate_json(copy)["tools"].toArray()[index].toObject();
        details->setPlainText(
            QJsonDocument(QJsonObject{{"routes", json["routes"]}, {"change", json["change"]}})
                .toJson(QJsonDocument::Indented));
    };
    auto refreshTools = [&] {
        toolList->clear();
        for (const auto &t : copy.tools)
            toolList->addItem(t.id + " · " + t.name);
        if (!copy.tools.empty())
            toolList->setCurrentRow(0);
    };

    auto storeTool = [&] {
        if (selected < 0)
            return;
        auto &t = copy.tools[selected];
        bool changed = std::abs(t.nozzle.diameter - diameter->value()) > 1e-9;
        t.name = toolName->text();
        t.controllerTool = toolNumber->value();
        t.duetDrive = drive->value();
        t.duetHeater = heaterIndex->value();
        t.extruder = extruder->text();
        t.heater = heater->text();
        t.useDummy = dummy->isChecked();
        t.detachable = detachable->isChecked();
        t.nozzle.diameter = diameter->value();
        t.nozzle.name = QString::number(t.nozzle.diameter) + " mm";
        t.nozzle.minHeight = minHeight->value();
        t.nozzle.maxHeight = maxHeight->value();
        t.nozzle.minWidth = minWidth->value();
        t.nozzle.maxWidth = maxWidth->value();
        t.nozzle.autoBeadLimits = automaticLimits->isChecked();
        t.nozzle.maxFlow = maxFlow->value();
        t.nozzle.calibrated = calibrated->isChecked();
        if (t.useDummy && (changed || t.dummy.components.empty()))
            t.dummy = make_dummy_toolhead(t.nozzle.diameter);
        auto &a = t.useDummy ? t.dummy : t.assembly;
        a.tcp = read_vector(tcp->text());
        a.axis = read_vector(axis->text());
        if (a.axis.norm() < 1e-8)
            throw std::runtime_error("Nozzle axis cannot be zero");
        a.mount.translation() = read_vector(mount->text());
        a.mount.linear() =
            Eigen::Quaterniond::FromTwoVectors(a.axis.normalized(), Vec3::UnitZ()).toRotationMatrix();
        t.dock.translation() = read_vector(dock->text());
        t.nozzle.dummyNozzle = nozzleMode->currentIndex() == 2;
        t.nozzle.nozzleMount.translation() = read_vector(nozzleMount->text());
        auto angles = read_vector(nozzleRotation->text()) * pi / 180;
        t.nozzle.nozzleMount.linear() =
            (rotation_z(angles.z()) * rotation_y(angles.y()) * rotation_x(angles.x())).linear();
        t.nozzle.separateNozzle.tcp = read_vector(separateTip->text());
        t.nozzle.separateNozzle.axis = read_vector(separateAxis->text());
        a.contactRadius = contactRadius->value();
        a.contactHeight = contactHeight->value();
        if (nozzleMode->currentIndex() == 0)
            t.nozzle.separateNozzle = {};
        for (int i = 0; i < components->rowCount(); ++i) {
            auto found = std::find_if(a.components.begin(), a.components.end(),
                                      [&](const Component &c) { return c.id == cell(components, i, 0); });
            if (found == a.components.end())
                continue;
            auto &c = *found;
            c.link = Link(static_cast<QComboBox *>(components->cellWidget(i, 2))->currentIndex());
            c.nozzle = static_cast<QCheckBox *>(components->cellWidget(i, 3))->isChecked();
            c.visualOnly = static_cast<QCheckBox *>(components->cellWidget(i, 4))->isChecked();
            if (c.nozzle)
                t.nozzle.componentId = c.id;
        }
        if (t.nozzle.calibrated)
            t.nozzle.geometryHash = t.assembly.sourceHash;
        QJsonParseError error;
        auto recipe = QJsonDocument::fromJson(details->toPlainText().toUtf8(), &error).object();
        if (error.error != QJsonParseError::NoError)
            throw std::runtime_error("Invalid tool recipe JSON");
        auto json = plate_json(copy);
        auto tools = json["tools"].toArray();
        auto entry = tools[selected].toObject();
        entry["routes"] = recipe["routes"];
        entry["change"] = recipe["change"];
        tools[selected] = entry;
        json["tools"] = tools;
        Project parsed;
        read_plate(parsed, json);
        copy.tools = parsed.tools;
    };
    auto fillLimits = [&] {
        NozzleProfile defaults;
        defaults.diameter = diameter->value();
        defaults.reset_bead_limits();
        minHeight->setValue(defaults.minHeight);
        maxHeight->setValue(defaults.maxHeight);
        minWidth->setValue(defaults.minWidth);
        maxWidth->setValue(defaults.maxWidth);
    };
    QObject::connect(diameter, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [&] {
        calibrated->setChecked(false);
        if (automaticLimits->isChecked())
            fillLimits();
    });
    QObject::connect(automaticLimits, &QCheckBox::toggled, &dialog, [&](bool enabled) {
        if (enabled)
            fillLimits();
    });
    QObject::connect(dummy, &QCheckBox::toggled, &dialog, [&](bool enabled) {
        if (selected < 0)
            return;
        try {
            QSignalBlocker toggleGuard(dummy);
            dummy->setChecked(!enabled);
            storeTool();
            dummy->setChecked(enabled);
            auto &tool = copy.tools[selected];
            tool.useDummy = enabled;
            if (enabled && tool.dummy.components.empty())
                tool.dummy = make_dummy_toolhead(tool.nozzle.diameter);
            if (enabled && nozzleMode->currentIndex() == 0) {
                auto nozzle = make_dummy_toolhead(tool.nozzle.diameter).components.front();
                for (auto &component : tool.dummy.components)
                    if (component.nozzle) {
                        component.mesh = nozzle.mesh;
                        component.renderMesh = nozzle.renderMesh;
                    }
            }
            loadTool(selected);
            calibrated->setChecked(false);
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Dummy geometry", e.what());
        }
    });
    QObject::connect(toolList, &QListWidget::currentRowChanged, &dialog, [&](int row) {
        if (row < 0)
            return;
        try {
            storeTool();
            loadTool(row);
        } catch (const std::exception &e) {
            QSignalBlocker guard(toolList);
            toolList->setCurrentRow(selected);
            QMessageBox::warning(&dialog, "Tool settings", e.what());
        }
    });
    QObject::connect(applyTool, &QPushButton::clicked, &dialog, [&] {
        try {
            storeTool();
            toolList->item(selected)->setText(copy.tools[selected].id + " · " + copy.tools[selected].name);
            loadTool(selected);
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Tool settings", e.what());
        }
    });
    QObject::connect(addTool, &QPushButton::clicked, &dialog, [&] {
        try {
            storeTool();
            ToolDefinition t;
            t.id = "tool-" + QString::number(copy.tools.size() + 1);
            t.name = "New tool";
            t.controllerTool = t.duetDrive = t.duetHeater = copy.tools.size();
            t.detachable = true;
            t.extruder = t.heater = "extruder" + QString::number(copy.tools.size());
            t.dummy = make_dummy_toolhead(.4);
            t.nozzle.reset_bead_limits();
            t.routes.push_back({copy.materials.front().id, "0", {}, true});
            copy.tools.push_back(t);
            refreshTools();
            toolList->setCurrentRow(copy.tools.size() - 1);
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    QObject::connect(importStep, &QPushButton::clicked, &dialog, [&] {
        try {
            if (selected < 0)
                return;
            auto file = QFileDialog::getOpenFileName(&dialog, "Toolhead STEP", {}, "STEP (*.step *.stp)");
            if (file.isEmpty())
                return;
            ToolheadAssembly a;
            if (import_step_dialog(&dialog, file, a, true)) {
                storeTool();
                copy.tools[selected].assembly = std::move(a);
                copy.tools[selected].useDummy = false;
                copy.tools[selected].nozzle.calibrated = false;
                loadTool(selected);
            }
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    QObject::connect(importNozzle, &QPushButton::clicked, &dialog, [&] {
        try {
            if (selected < 0)
                return;
            auto file =
                QFileDialog::getOpenFileName(&dialog, "Installed nozzle STEP", {}, "STEP (*.step *.stp)");
            if (file.isEmpty())
                return;
            ToolheadAssembly a;
            if (import_step_dialog(&dialog, file, a, true)) {
                storeTool();
                for (size_t i = 0; i < a.components.size(); ++i) {
                    auto &c = a.components[i];
                    c.nozzle = i == 0;
                    c.link = Link::Nozzle;
                }
                copy.tools[selected].nozzle.separateNozzle = std::move(a);
                copy.tools[selected].nozzle.calibrated = false;
                loadTool(selected);
            }
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate settings", e.what());
        }
    });
    tabs->addTab(toolPage, "Tools and nozzles");
    refreshTools();
    auto assignmentPage = new QWidget;
    auto al = new QVBoxLayout(assignmentPage);
    auto assignmentObject = new QComboBox;
    for (auto &o : copy.objects)
        assignmentObject->addItem(o.id);
    al->addWidget(assignmentObject);
    auto assignments = new QPlainTextEdit;
    assignments->setObjectName("materialAssignmentsJson");
    al->addWidget(new QLabel("Object/role/paint assignments are saved here; surface painting is also "
                             "available directly in Prepare. Volume modifiers use object coordinates."));
    al->addWidget(assignments);
    auto importVolume = new QPushButton("Add material volume from geometry");
    al->addWidget(importVolume);
    tabs->addTab(assignmentPage, "Assignments");
    int assignmentSelected = 0;
    auto storeAssignments = [&] {
        QJsonParseError error;
        auto document = QJsonDocument::fromJson(assignments->toPlainText().toUtf8(), &error);
        if (error.error != QJsonParseError::NoError || !document.isArray())
            throw std::runtime_error("Invalid assignment JSON");
        auto json = plate_json(copy);
        auto array = json["objects"].toArray();
        auto entry = array[assignmentSelected].toObject();
        entry["assignments"] = document.array();
        array[assignmentSelected] = entry;
        json["objects"] = array;
        Project parsed;
        read_plate(parsed, json);
        copy.objects[assignmentSelected].assignments = parsed.objects[assignmentSelected].assignments;
    };
    auto loadAssignments = [&](int row) {
        assignmentSelected = row;
        if (row < 0 || row >= int(copy.objects.size()))
            return;
        auto entry = plate_json(copy)["objects"].toArray()[row].toObject();
        assignments->setPlainText(
            QJsonDocument(entry["assignments"].toArray()).toJson(QJsonDocument::Indented));
    };
    QObject::connect(assignmentObject, &QComboBox::currentIndexChanged, &dialog, [&](int row) {
        try {
            storeAssignments();
            loadAssignments(row);
        } catch (const std::exception &e) {
            QSignalBlocker guard(assignmentObject);
            assignmentObject->setCurrentIndex(assignmentSelected);
            QMessageBox::warning(&dialog, "Assignments", e.what());
        }
    });
    loadAssignments(0);
    QObject::connect(importVolume, &QPushButton::clicked, &dialog, [&] {
        int row = assignmentObject->currentIndex();
        if (row < 0)
            return;
        auto file = QFileDialog::getOpenFileName(&dialog, "Material modifier volume", {},
                                                 "Geometry (*.stl *.obj *.3mf *.step *.stp)");
        if (file.isEmpty())
            return;
        try {
            storeAssignments();
            MaterialAssignment a;
            a.id = "volume-" + QString::number(copy.objects[row].assignments.size() + 1);
            a.kind = AssignmentKind::Volume;
            a.volume = load_mesh(file);
            a.materialId = copy.materials.back().id;
            copy.objects[row].assignments.push_back(a);
            loadAssignments(row);
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Modifier", e.what());
        }
    });
    auto assignRow = new QHBoxLayout;
    auto assignMaterial = new QLineEdit(copy.materials.front().id);
    auto assignTool = new QLineEdit;
    auto assignRole = new QComboBox;
    assignRole->addItems({"Whole object", "Walls/skin", "Support body", "Support interface"});
    auto addAssignment = new QPushButton("Add assignment");
    assignRow->addWidget(assignMaterial);
    assignRow->addWidget(assignTool);
    assignRow->addWidget(assignRole);
    assignRow->addWidget(addAssignment);
    al->addLayout(assignRow);
    QObject::connect(addAssignment, &QPushButton::clicked, &dialog, [&] {
        try {
            storeAssignments();
            auto &o = copy.objects[assignmentSelected];
            MaterialAssignment a;
            a.id = "assignment-" + QString::number(o.assignments.size() + 1);
            a.materialId = assignMaterial->text();
            a.toolId = assignTool->text();
            if (assignRole->currentIndex() == 0)
                a.kind = AssignmentKind::Object;
            else {
                a.kind = AssignmentKind::Role;
                a.role = assignRole->currentIndex() == 1   ? Role::OuterWall
                         : assignRole->currentIndex() == 2 ? Role::Support
                                                           : Role::SupportInterface;
            }
            o.assignments.push_back(a);
            loadAssignments(assignmentSelected);
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Assignment", e.what());
        }
    });
    auto planningPage = new QWidget;
    auto pl = new QFormLayout(planningPage);
    auto bed = number(copy.plate.bedTemperature, 0, 150), towerWidth = number(copy.plate.towerWidth, 1, 200);
    auto purgePosition = new QLineEdit(vector_text(copy.plate.purgePosition));
    auto tower = new QCheckBox("Generate purge tower");
    tower->setChecked(copy.plate.purgeTower);
    auto support = new QLineEdit(copy.plate.supportMaterial),
         interfaceMaterial = new QLineEdit(copy.plate.interfaceMaterial);
    auto purge = new QPlainTextEdit;
    purge->setObjectName("purgeTransitionsJson");
    purge->setPlainText(QJsonDocument(plate_json(copy)["settings"].toObject()["purge"].toArray())
                            .toJson(QJsonDocument::Indented));
    auto carriage = new QPlainTextEdit;
    carriage->setPlainText(
        QJsonDocument(plate_json(copy)["carriage"].toObject()).toJson(QJsonDocument::Indented));
    carriage->setMaximumHeight(180);
    auto capacity = number(copy.plate.purgeCapacity, 0, 1e9);
    pl->addRow("Station waste capacity · mm³", capacity);
    auto importCarriage = new QPushButton("Import shared carriage/coupler STEP");
    pl->addRow(importCarriage);
    QObject::connect(importCarriage, &QPushButton::clicked, &dialog, [&] {
        auto file = QFileDialog::getOpenFileName(&dialog, "Shared carriage STEP", {}, "STEP (*.step *.stp)");
        if (file.isEmpty())
            return;
        ToolheadAssembly assembly;
        if (import_step_dialog(&dialog, file, assembly, true)) {
            for (auto &c : assembly.components) {
                c.nozzle = false;
                c.link = Link::Carriage;
            }
            copy.carriageAssembly = assembly;
            carriage->setPlainText(
                QJsonDocument(plate_json(copy)["carriage"].toObject()).toJson(QJsonDocument::Indented));
        }
    });
    pl->addRow("Plate bed temperature", bed);
    pl->addRow("Support body material ID", support);
    pl->addRow("Support interface material ID", interfaceMaterial);
    pl->addRow(tower);
    pl->addRow("Tower width", towerWidth);
    pl->addRow("Purge position · x,y,z", purgePosition);
    pl->addRow("Purge matrix · volumes in mm³", purge);
    pl->addRow("Shared empty carriage/coupler CAD model · JSON", carriage);
    tabs->addTab(planningPage, "Purge and carriage");
    auto computePage = new QWidget;
    auto cf = new QFormLayout(computePage);
    auto backend = new QComboBox;
    backend->addItems({"Auto", "CPU", "NVIDIA CUDA", "Vulkan"});
    backend->setCurrentIndex(int(copy.compute.backend));
    backend->setObjectName("computeBackend");
    auto threads = new QSpinBox;
    threads->setRange(0, 1024);
    threads->setSpecialValueText("Auto · logical processors minus one");
    threads->setValue(copy.compute.threads);
    threads->setObjectName("computeThreads");
    auto devices = new QComboBox;
    devices->addItem("Automatic", QString{});
    for (auto device : compute_devices())
        if (device != "CPU")
            devices->addItem(device, device.section(": ", 1));
    int deviceIndex = devices->findData(copy.compute.device);
    if (deviceIndex >= 0)
        devices->setCurrentIndex(deviceIndex);
    auto memory = number(copy.compute.memoryFraction, .01, .9);
    cf->addRow("Backend", backend);
    cf->addRow("CPU thread budget", threads);
    cf->addRow("GPU device", devices);
    cf->addRow("GPU memory fraction", memory);
    cf->addRow(new QLabel(
        "Unsupported devices and failed batches fall back to CPU. Final collision checks remain CPU-owned."));
    tabs->addTab(computePage, "Performance");
    auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    outer->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        try {
            storeTool();
            storeMaterials();
            storeObjects();
            auto json = plate_json(copy);
            QJsonParseError error;
            auto assignmentDoc = QJsonDocument::fromJson(assignments->toPlainText().toUtf8(), &error);
            if (error.error != QJsonParseError::NoError || !assignmentDoc.isArray())
                throw std::runtime_error("Invalid assignment JSON");
            auto array = json["objects"].toArray();
            auto selectedObject = array[assignmentObject->currentIndex()].toObject();
            selectedObject["assignments"] = assignmentDoc.array();
            array[assignmentObject->currentIndex()] = selectedObject;
            json["objects"] = array;
            auto settings = json["settings"].toObject();
            settings["bedTemperature"] = bed->value();
            settings["supportMaterial"] = support->text();
            settings["interfaceMaterial"] = interfaceMaterial->text();
            settings["purgeTower"] = tower->isChecked();
            settings["towerWidth"] = towerWidth->value();
            settings["purgeCapacity"] = capacity->value();
            auto v = read_vector(purgePosition->text());
            settings["purgePosition"] = QJsonArray{v.x(), v.y(), v.z()};
            auto purgeDoc = QJsonDocument::fromJson(purge->toPlainText().toUtf8(), &error);
            if (error.error != QJsonParseError::NoError || !purgeDoc.isArray())
                throw std::runtime_error("Invalid purge matrix");
            settings["purge"] = purgeDoc.array();
            json["settings"] = settings;
            auto carriageDoc = QJsonDocument::fromJson(carriage->toPlainText().toUtf8(), &error);
            if (error.error != QJsonParseError::NoError || !carriageDoc.isObject())
                throw std::runtime_error("Invalid carriage model");
            json["carriage"] = carriageDoc.object();
            Project parsed;
            read_plate(parsed, json);
            copy.objects = parsed.objects;
            copy.plate = parsed.plate;
            copy.carriageAssembly = parsed.carriageAssembly;
            copy.plate.sequence = PrintSequence(sequence->currentIndex());
            copy.plate.automaticOrder = automaticOrder->isChecked();
            copy.plate.automaticTools = automaticTools->isChecked();
            copy.plate.order.clear();
            for (int i = 0; i < order->count(); ++i)
                copy.plate.order.push_back(order->item(i)->text());
            copy.compute.backend = ComputeBackend(backend->currentIndex());
            copy.compute.threads = threads->value();
            copy.compute.device = devices->currentData().toString();
            copy.compute.memoryFraction = memory->value();
            validate_plate_setup(copy);
            copy.path.clear();
            copy.events.clear();
            copy.invalidate();
            synchronize_plate_preview(copy);
            project = std::move(copy);
            dialog.accept();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Plate configuration", e.what());
        }
    });
    return dialog.exec() == QDialog::Accepted;
}
