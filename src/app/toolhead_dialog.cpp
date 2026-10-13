#include "core/plate.h"
#include "machine_dialog.h"
#include "printer_widgets.h"
#include "viewport.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>
using namespace fiveaxis;
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
static bool calibrate(Project &project, QWidget *parent) {
    QDialog dialog(parent);
    dialog.setWindowTitle("Machine setup · STEP toolhead");
    dialog.resize(1000, 720);
    auto horizontal = new QHBoxLayout(&dialog);
    auto formWidget = new QWidget;
    auto left = new QVBoxLayout(formWidget);
    auto form = new QFormLayout;
    left->addLayout(form);
    Project copy = project;
    left->addWidget(
        new QLabel("Source STEP length units: " +
                   (copy.active_toolhead().sourceUnits.isEmpty() ? QString("synthetic mm")
                                                                 : copy.active_toolhead().sourceUnits) +
                   " · converted to mm"));
    auto tcp = new QLineEdit(vector_text(copy.active_toolhead().tcp)),
         axis = new QLineEdit(vector_text(copy.active_toolhead().axis)),
         mount = new QLineEdit(vector_text(copy.active_toolhead().mount.translation())),
         bed = new QLineEdit(vector_text(copy.machine.bedPivot)),
         pa = new QLineEdit(vector_text(copy.machine.headPivotA)),
         pb = new QLineEdit(vector_text(copy.machine.headPivotB)),
         minimum = new QLineEdit(vector_text(copy.machine.minXYZ)),
         maximum = new QLineEdit(vector_text(copy.machine.maxXYZ));
    auto clearance = spin(copy.machine.clearance, 0, 10, 3),
         radius = spin(copy.active_toolhead().contactRadius, .01, 5, 3),
         height = spin(copy.active_toolhead().contactHeight, .01, 5, 3);
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
    auto table = new QTableWidget(int(copy.active_toolhead().components.size()), 4);
    table->setHorizontalHeaderLabels({"Component / instance", "Moving link", "Nozzle", "Visual only"});
    std::vector<QComboBox *> links;
    std::vector<QCheckBox *> nozzles, visuals;
    for (size_t i = 0; i < copy.active_toolhead().components.size(); ++i) {
        auto &c = copy.active_toolhead().components[i];
        table->setItem(i, 0, new QTableWidgetItem(c.name + " · " + c.id));
        auto link = new QComboBox;
        link->addItems({"Carriage", "Rotary stage", "Nozzle assembly"});
        link->setCurrentIndex(int(c.link));
        links.push_back(link);
        table->setCellWidget(i, 1, link);
        auto nozzle = new PrinterCheckBox;
        nozzle->setChecked(c.nozzle);
        nozzles.push_back(nozzle);
        table->setCellWidget(i, 2, nozzle);
        auto visual = new PrinterCheckBox;
        visual->setChecked(c.visualOnly);
        visuals.push_back(visual);
        table->setCellWidget(i, 3, visual);
    }
    table->resizeColumnsToContents();
    left->addWidget(table, 1);
    auto interfaces = new QTextEdit;
    interfaces->setMaximumHeight(75);
    interfaces->setPlaceholderText("First id | second id | CAD center x,y,z | box size x,y,z");
    for (const auto &rule : copy.active_toolhead().interfaces)
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
    angleA->setEnabled(copy.machine.rotary_axes());
    angleB->setEnabled(copy.machine.rotary_axes());
    previewColumn->addWidget(new QLabel("Calibration pose · A / B or C degrees"));
    previewColumn->addWidget(angleA);
    previewColumn->addWidget(angleB);
    copy.path.clear();
    Sample pose;
    pose.joint.xyz = Vec3(0, 0, 15);
    copy.path.push_back(pose);
    preview->set_project(&copy);
    preview->fit();
    QObject::connect(pick, &QPushButton::clicked, preview, [preview] { preview->set_tool_pick(true); });
    QObject::connect(preview, &Viewport::tool_picked, &dialog, [tcp, nozzles](int index, Vec3 point) {
        tcp->setText(vector_text(point));
        for (size_t i = 0; i < nozzles.size(); ++i)
            nozzles[i]->setChecked(int(i) == index);
    });
    auto updatePose = [&] {
        try {
            copy.active_toolhead().tcp = parse_vector(tcp->text());
            auto n = parse_vector(axis->text());
            if (n.norm() < 1e-8)
                return;
            copy.active_toolhead().axis = n.normalized();
            copy.active_toolhead().mount = translation(parse_vector(mount->text()));
            copy.active_toolhead().mount.linear() =
                Eigen::Quaterniond::FromTwoVectors(copy.active_toolhead().axis, Vec3::UnitZ())
                    .toRotationMatrix();
            copy.machine.bedPivot = parse_vector(bed->text());
            copy.machine.headPivotA = parse_vector(pa->text());
            copy.machine.headPivotB = parse_vector(pb->text());
            for (size_t i = 0; i < links.size(); ++i)
                copy.active_toolhead().components[i].link = Link(links[i]->currentIndex());
        } catch (...) {
            return;
        }
        copy.path.front().joint.a = angleA->value() * pi / 180;
        copy.path.front().joint.b = angleB->value() * pi / 180;
        preview->set_time(0);
    };
    QObject::connect(angleA, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog,
                     [&](double) { updatePose(); });
    QObject::connect(angleB, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog,
                     [&](double) { updatePose(); });
    for (auto field : {tcp, axis, mount, bed, pa, pb})
        QObject::connect(field, &QLineEdit::textChanged, &dialog, [&](const QString &) { updatePose(); });
    for (auto field : links)
        QObject::connect(field, &QComboBox::currentIndexChanged, &dialog, [&](int) { updatePose(); });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        try {
            ToolheadAssembly assembly = project.active_toolhead();
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
            project.active_toolhead() = std::move(assembly);
            if (project.plateEnabled && !project.tools.empty()) {
                auto &tool = project.tools[0];
                auto &base = tool.useDummy ? tool.dummy : tool.assembly;
                const auto &edited = project.active_toolhead();
                base.mount = edited.mount;
                base.axis = edited.axis;
                base.tcp = edited.tcp;
                base.contactRadius = edited.contactRadius;
                base.contactHeight = edited.contactHeight;
                base.interfaces = edited.interfaces;
                for (auto &component : base.components)
                    for (const auto &value : edited.components)
                        if (component.id == value.id) {
                            component.link = value.link;
                            component.nozzle = value.nozzle;
                            component.visualOnly = value.visualOnly;
                        }
                if (tool.nozzle.dummyNozzle || !tool.nozzle.separateNozzle.components.empty()) {
                    auto source = tool.nozzle.dummyNozzle ? make_dummy_toolhead(tool.nozzle.diameter)
                                                          : tool.nozzle.separateNozzle;
                    tool.nozzle.nozzleMount.linear() =
                        Eigen::Quaterniond::FromTwoVectors(source.axis.normalized(), edited.axis.normalized())
                            .toRotationMatrix();
                    tool.nozzle.nozzleMount.translation() =
                        edited.tcp - tool.nozzle.nozzleMount.linear() * source.tcp;
                }
                tool.nozzle.calibrated = true;
                tool.nozzle.geometryHash = base.sourceHash;
            }
            project.machine.bedPivot = parse_vector(bed->text());
            project.machine.headPivotA = parse_vector(pa->text());
            project.machine.headPivotB = parse_vector(pb->text());
            project.machine.minXYZ = parse_vector(minimum->text());
            project.machine.maxXYZ = parse_vector(maximum->text());
            project.machine.clearance = clearance->value();
            project.invalidate();
            dialog.accept();
        } catch (const std::exception &e) {
            QMessageBox::warning(&dialog, "Setup", e.what());
        }
    });
    return dialog.exec() == QDialog::Accepted;
}

bool edit_toolhead(Project &project, QWidget *parent, const QString &toolId) {
    if (!project.plateEnabled || project.tools.empty())
        return calibrate(project, parent);
    auto found = std::find_if(project.tools.begin(), project.tools.end(),
                              [&](const auto &tool) { return tool.id == toolId; });
    if (found == project.tools.end())
        return false;
    Project proxy = project;
    proxy.plateEnabled = false;
    proxy.useDummyToolhead = false;
    proxy.toolhead = installed_assembly(*found);
    if (!calibrate(proxy, parent))
        return false;
    auto &base = found->useDummy ? found->dummy : found->assembly;
    const auto &edited = proxy.toolhead;
    base.mount = edited.mount;
    base.tcp = edited.tcp;
    base.axis = edited.axis;
    base.contactRadius = edited.contactRadius;
    base.contactHeight = edited.contactHeight;
    base.interfaces = edited.interfaces;
    for (auto &component : base.components)
        for (const auto &value : edited.components)
            if (component.id == value.id) {
                component.link = value.link;
                component.nozzle = value.nozzle;
                component.visualOnly = value.visualOnly;
            }
    if (found->nozzle.dummyNozzle || !found->nozzle.separateNozzle.components.empty()) {
        auto source = found->nozzle.dummyNozzle ? make_dummy_toolhead(found->nozzle.diameter)
                                                : found->nozzle.separateNozzle;
        found->nozzle.nozzleMount.linear() =
            Eigen::Quaterniond::FromTwoVectors(source.axis.normalized(), edited.axis.normalized())
                .toRotationMatrix();
        found->nozzle.nozzleMount.translation() =
            edited.tcp - found->nozzle.nozzleMount.linear() * source.tcp;
    }
    found->nozzle.calibrated = true;
    found->nozzle.geometryHash = base.sourceHash;
    project.machine = proxy.machine;
    synchronize_plate_preview(project);
    project.invalidate();
    return true;
}
