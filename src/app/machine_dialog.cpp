#include "machine_dialog.h"
#include "core/geometry.h"
#include "core/machine_settings.h"
#include "core/plate.h"
#include "core/project.h"
#include "plate_editor.h"
#include "printer_widgets.h"
#include "step_dialog.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QStyle>
#include <QTabWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <functional>
#include <stdexcept>
#include <tuple>
using namespace fiveaxis;
namespace {
void light_palette(QWidget &widget) {
    auto light = widget.style()->standardPalette();
    light.setColor(QPalette::Window, Qt::white);
    light.setColor(QPalette::Base, Qt::white);
    light.setColor(QPalette::WindowText, QColor("#30383c"));
    light.setColor(QPalette::Text, QColor("#30383c"));
    light.setColor(QPalette::Button, QColor("#fafbfb"));
    light.setColor(QPalette::ButtonText, QColor("#30383c"));
    light.setColor(QPalette::Highlight, QColor("#00a64b"));
    light.setColor(QPalette::HighlightedText, Qt::white);
    widget.setPalette(light);
}
const char *editorStyle = R"(
QDialog { background: white; color: #30383c; }
QWidget { color: #30383c; font-size: 14px; }
QTabWidget::pane { border: 0; border-top: 1px solid #d9dddf; background: white; }
QTabBar::tab { padding: 12px 16px 8px 16px; background: white; color: #667076; border-bottom: 3px solid transparent; }
QTabBar::tab:selected { color: #263238; font-weight: bold; border-bottom: 3px solid #00b352; }
QGroupBox { border: 0; border-top: 1px solid #e7eaec; margin-top: 23px; padding-top: 15px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 0; padding: 0 10px 0 0; background: white; }
QLineEdit, QComboBox, QDoubleSpinBox, QPlainTextEdit { background: white; border: 1px solid #d3d9dc; padding: 3px; selection-background-color: #00a64b; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QPlainTextEdit:focus { border: 1px solid #00a64b; }
QPushButton { background: #fafbfb; border: 1px solid #cbd2d5; border-radius: 4px; padding: 7px 14px; }
QPushButton:hover { border-color: #00a64b; }
QPushButton:default { background: #00b352; color: white; border-color: #00b352; }
QScrollBar:vertical { background: #f4f5f6; width: 12px; }
QScrollBar::handle:vertical { background: #a8b1b5; min-height: 25px; border-radius: 4px; }
QToolButton { background: white; border: 0; padding: 6px; }
QScrollArea { border: 0; background: white; }
QWidget#page { background: white; }
QLabel#hint { color: #69767c; font-size: 12px; }
QDoubleSpinBox:disabled, QLineEdit:disabled { background: #f0f2f3; color: #879196; }
)";
QDoubleSpinBox *number(double value, double lo, double hi, const QString &unit, const QString &name) {
    auto spin = new QDoubleSpinBox;
    spin->setObjectName(name);
    spin->setRange(lo, hi);
    spin->setDecimals(unit == "mm" ? 4 : 2);
    spin->setSingleStep(unit == "mm" ? .1 : 1);
    spin->setValue(value);
    if (!unit.isEmpty())
        spin->setSuffix("  " + unit);
    spin->setMinimumWidth(170);
    spin->setMaximumWidth(180);
    spin->setKeyboardTracking(false);
    return spin;
}
QJsonArray parse_polygon(const QString &text) {
    QJsonArray points;
    for (auto token : text.split(',', Qt::SkipEmptyParts)) {
        auto xy = token.trimmed().split(QRegularExpression("[xX\\s]+"), Qt::SkipEmptyParts);
        if (xy.size() != 2)
            throw std::runtime_error("Enter polygon vertices as XxY, XxY, XxY");
        bool xok, yok;
        double x = xy[0].toDouble(&xok), y = xy[1].toDouble(&yok);
        if (!xok || !yok || !std::isfinite(x + y))
            throw std::runtime_error("Invalid polygon coordinates");
        points.append(QJsonArray{x, y});
    }
    return points;
}
QString polygon_text(const QJsonArray &points) {
    QStringList list;
    for (const auto &entry : points) {
        auto v = entry.toArray();
        list << QString("%1x%2").arg(v[0].toDouble()).arg(v[1].toDouble());
    }
    return list.join(", ");
}
class BedPreview : public QWidget {
  public:
    QJsonObject settings;
    BedPreview() {
        setMinimumSize(350, 350);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

  protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), Qt::white);
        try {
            auto outline = bed_outline(settings);
            Eigen::AlignedBox3d bounds;
            for (auto p : outline)
                bounds.extend(p);
            QRectF frame(36, 20, width() - 60, height() - 60);
            double scale = std::min(frame.width() / bounds.sizes().x(), frame.height() / bounds.sizes().y());
            QPointF offset(frame.center().x() - scale * bounds.center().x(),
                           frame.center().y() + scale * bounds.center().y());
            auto map = [&](const Vec3 &v) {
                return QPointF(offset.x() + v.x() * scale, offset.y() - v.y() * scale);
            };
            QPolygonF poly;
            for (auto v : outline)
                poly << map(v);
            QPainterPath clip;
            clip.addPolygon(poly);
            painter.save();
            painter.setClipPath(clip);
            painter.fillPath(clip, QColor("#fafbfb"));
            auto image =
                QImage::fromData(QByteArray::fromBase64(settings["bedTexture"].toString().toLatin1()));
            if (!image.isNull())
                painter.drawImage(poly.boundingRect(), image);
            painter.setPen(QPen(QColor("#dfe3e5"), 1));
            for (double x = std::floor(bounds.min().x() / 10) * 10; x <= bounds.max().x(); x += 10)
                painter.drawLine(map(Vec3(x, bounds.min().y(), 0)), map(Vec3(x, bounds.max().y(), 0)));
            for (double y = std::floor(bounds.min().y() / 10) * 10; y <= bounds.max().y(); y += 10)
                painter.drawLine(map(Vec3(bounds.min().x(), y, 0)), map(Vec3(bounds.max().x(), y, 0)));
            for (const auto &entry : settings["bedExclusions"].toArray()) {
                QPolygonF excluded;
                for (const auto &vertex : entry.toArray()) {
                    auto v = vertex.toArray();
                    excluded << map(Vec3(v[0].toDouble(), v[1].toDouble(), 0));
                }
                painter.setBrush(QColor(235, 86, 66, 70));
                painter.setPen(QColor("#c64236"));
                painter.drawPolygon(excluded);
            }
            painter.restore();
            painter.setPen(QPen(QColor("#48575d"), 1.3));
            painter.setBrush(Qt::NoBrush);
            painter.drawPolygon(poly);
            QPointF zero = map(Vec3::Zero());
            painter.setPen(QPen(Qt::red, 2));
            painter.drawLine(zero, zero + QPointF(28, 0));
            painter.setPen(QPen(QColor("#00b352"), 2));
            painter.drawLine(zero, zero + QPointF(0, -28));
            painter.setPen(QColor("#30383c"));
            painter.drawText(zero + QPointF(3, 16), "(0, 0)");
        } catch (const std::exception &error) {
            painter.drawText(rect(), Qt::AlignCenter, error.what());
        }
    }
};
bool edit_bed(Project &p, QWidget *parent) {
    QDialog dialog(parent);
    dialog.setObjectName("bedShapeDialog");
    dialog.setWindowTitle("Bed shape");
    dialog.resize(1050, 720);
    dialog.setStyleSheet(editorStyle);
    light_palette(dialog);
    auto settings = p.machine.printer;
    auto layout = new QVBoxLayout(&dialog), controls = new QVBoxLayout;
    auto columns = new QHBoxLayout;
    layout->addLayout(columns, 1);
    columns->addLayout(controls);
    auto shape = new QComboBox;
    shape->setObjectName("bedShape");
    shape->addItems({"Rectangular", "Circular", "Custom polygon"});
    shape->setCurrentIndex(settings["bedShape"].toString() == "circle"   ? 1
                           : settings["bedShape"].toString() == "custom" ? 2
                                                                         : 0);
    auto form = new QFormLayout;
    controls->addLayout(form);
    form->addRow("Shape", shape);
    auto width = number(printer_number(settings, "bedWidth", 320), 1, 10000, "mm", "bedWidth");
    auto depth = number(printer_number(settings, "bedDepth", 320), 1, 10000, "mm", "bedDepth");
    auto ox = number(printer_number(settings, "bedOriginX", 160), -10000, 10000, "mm", "bedOriginX");
    auto oy = number(printer_number(settings, "bedOriginY", 160), -10000, 10000, "mm", "bedOriginY");
    for (auto pair : {std::make_pair("Size X", width), {"Size Y", depth}, {"Origin X", ox}, {"Origin Y", oy}})
        form->addRow(pair.first, pair.second);
    auto poly = new QLineEdit(polygon_text(settings["bedPolygon"].toArray()));
    poly->setPlaceholderText("0x0, 256x0, 256x256, 0x256");
    poly->setObjectName("bedPolygon");
    form->addRow("Vertices", poly);
    auto preview = new BedPreview;
    columns->addWidget(preview, 1);
    auto update = [&] {
        settings["bedShape"] = QStringList{"rectangle", "circle", "custom"}[shape->currentIndex()];
        if (shape->currentIndex() == 1) {
            QSignalBlocker blocker(depth);
            depth->setValue(width->value());
        }
        settings["bedWidth"] = width->value();
        settings["bedDepth"] = depth->value();
        settings["bedOriginX"] = ox->value();
        settings["bedOriginY"] = oy->value();
        try {
            settings["bedPolygon"] = parse_polygon(poly->text());
        } catch (...) {
        }
        form->setRowVisible(poly, shape->currentIndex() == 2);
        depth->setEnabled(shape->currentIndex() == 0);
        preview->settings = settings;
        preview->update();
    };
    QObject::connect(shape, &QComboBox::currentIndexChanged, &dialog, [&](int) { update(); });
    for (auto field : {width, depth, ox, oy})
        QObject::connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog,
                         [&](double) { update(); });
    QObject::connect(poly, &QLineEdit::textChanged, &dialog, [&](const QString &) { update(); });
    auto textureGroup = new QGroupBox("Texture");
    auto textureLayout = new QVBoxLayout(textureGroup);
    controls->addWidget(textureGroup);
    auto textureName = new QLabel(settings["bedTextureName"].toString("None"));
    textureName->setWordWrap(true);
    textureLayout->addWidget(textureName);
    auto textureLoad = new QPushButton("Load…"), textureRemove = new QPushButton("Remove");
    textureLayout->addWidget(textureLoad);
    textureLayout->addWidget(textureRemove);
    QObject::connect(textureLoad, &QPushButton::clicked, &dialog, [&] {
        auto file = QFileDialog::getOpenFileName(&dialog, "Bed texture", {}, "Images (*.png *.jpg *.bmp)");
        if (file.isEmpty())
            return;
        QFile data(file);
        if (!data.open(QIODevice::ReadOnly) || data.size() > 16000000) {
            QMessageBox::warning(&dialog, "Texture", "Choose an image smaller than 16 MB");
            return;
        }
        auto bytes = data.readAll();
        if (QImage::fromData(bytes).isNull()) {
            QMessageBox::warning(&dialog, "Texture", "Cannot decode image");
            return;
        }
        settings["bedTexture"] = QString::fromLatin1(bytes.toBase64());
        settings["bedTextureName"] = QFileInfo(file).fileName();
        textureName->setText(settings["bedTextureName"].toString());
        update();
    });
    QObject::connect(textureRemove, &QPushButton::clicked, &dialog, [&] {
        settings.remove("bedTexture");
        settings.remove("bedTextureName");
        textureName->setText("None");
        update();
    });
    auto modelGroup = new QGroupBox("Model · collision geometry");
    auto modelLayout = new QVBoxLayout(modelGroup);
    controls->addWidget(modelGroup);
    auto modelName = new QLabel(settings["bedModelName"].toString("Generated from bed shape"));
    modelName->setWordWrap(true);
    modelLayout->addWidget(modelName);
    auto modelLoad = new QPushButton("Load STEP / STL…"), modelRemove = new QPushButton("Use generated bed");
    modelLayout->addWidget(modelLoad);
    modelLayout->addWidget(modelRemove);
    std::optional<Fixture> model;
    QObject::connect(modelLoad, &QPushButton::clicked, &dialog, [&] {
        auto file =
            QFileDialog::getOpenFileName(&dialog, "Bed collision model · calibrated plate coordinates", {},
                                         "Geometry (*.step *.stp *.stl *.obj)");
        if (file.isEmpty())
            return;
        try {
            Fixture f;
            f.name = "printer-bed";
            f.movesWithBed = true;
            if (file.endsWith(".step", Qt::CaseInsensitive) || file.endsWith(".stp", Qt::CaseInsensitive)) {
                ToolheadAssembly assembly;
                if (!import_step_dialog(&dialog, file, assembly, false))
                    return;
                f.sourceStep = assembly.sourceStep;
                for (const auto &component : assembly.components) {
                    int offset = int(f.mesh.vertices.size());
                    f.mesh.vertices.insert(f.mesh.vertices.end(), component.mesh.vertices.begin(),
                                           component.mesh.vertices.end());
                    for (auto triangle : component.mesh.triangles)
                        f.mesh.triangles.push_back(triangle + Eigen::Vector3i::Constant(offset));
                    f.mesh.error = std::max(f.mesh.error, component.mesh.error);
                }
            } else
                f.mesh = load_mesh(file);
            if (f.mesh.empty())
                throw std::runtime_error("Empty bed model");
            model = std::move(f);
            settings["bedModelName"] = QFileInfo(file).fileName();
            settings["bedCustomModel"] = true;
            modelName->setText(settings["bedModelName"].toString());
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Bed model", error.what());
        }
    });
    QObject::connect(modelRemove, &QPushButton::clicked, &dialog, [&] {
        model.reset();
        settings.remove("bedModelName");
        settings["bedCustomModel"] = false;
        modelName->setText("Generated from bed shape");
    });
    auto hint = new QLabel("The origin locates machine (0, 0) within the plate. Imported bed models must "
                           "already use calibrated plate coordinates.");
    hint->setWordWrap(true);
    hint->setObjectName("hint");
    controls->addWidget(hint);
    controls->addStretch();
    auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttons);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        try {
            update();
            if (shape->currentIndex() == 2)
                settings["bedPolygon"] = parse_polygon(poly->text());
            settings["bedEnabled"] = true;
            MachineProfile machine = p.machine;
            machine.printer = settings;
            validate_printer_settings(machine);
            if (!settings["bedCustomModel"].toBool())
                model = Fixture{"printer-bed", printer_bed_mesh(settings), true, {}};
            if (model) {
                p.fixtures.erase(std::remove_if(p.fixtures.begin(), p.fixtures.end(),
                                                [](const auto &f) {
                                                    return f.name == "printer-bed" ||
                                                           (f.movesWithBed &&
                                                            f.name.contains("bed", Qt::CaseInsensitive));
                                                }),
                                 p.fixtures.end());
                p.fixtures.push_back(*model);
            }
            p.machine.printer = settings;
            dialog.accept();
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Bed shape", error.what());
        }
    });
    update();
    return dialog.exec() == QDialog::Accepted;
}
struct Row {
    QWidget *label, *field;
    QString text;
    bool advanced;
};
class AdvancedSwitch : public QCheckBox {
  public:
    AdvancedSwitch() : QCheckBox("Advanced") {
        setMinimumSize(140, 32);
    }

  protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.setPen(QColor("#30383c"));
        p.drawText(QRect(0, 0, width() - 46, height()), Qt::AlignVCenter, text());
        QRectF track(width() - 42, height() / 2.0 - 9, 38, 18);
        p.setPen(Qt::NoPen);
        p.setBrush(isChecked() ? QColor("#00b352") : QColor("#aeb3b5"));
        p.drawRoundedRect(track, 9, 9);
        p.setBrush(Qt::white);
        p.drawEllipse(QPointF(isChecked() ? track.right() - 9 : track.left() + 9, track.center().y()), 7, 7);
        if (hasFocus()) {
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(QColor("#007c39"), 1, Qt::DotLine));
            p.drawRect(rect().adjusted(1, 1, -1, -1));
        }
    }
    bool hitButton(const QPoint &point) const override {
        return rect().contains(point);
    }
};
} // namespace

bool edit_machine(Project &project, QWidget *parent) {
    Project copy = project;
    if (!copy.plateEnabled)
        enable_plate(copy);
    QDialog dialog(parent);
    dialog.setObjectName("machineSettingsDialog");
    dialog.setWindowTitle("Printer settings · FiveAxisLab");
    dialog.resize(1140, 900);
    dialog.setMinimumSize(860, 600);
    dialog.setStyleSheet(editorStyle);
    light_palette(dialog);
    auto root = new QVBoxLayout(&dialog);
    root->setContentsMargins(22, 16, 22, 16);
    root->setSpacing(8);
    auto bar = new QHBoxLayout;
    root->addLayout(bar);
    auto profile = new QComboBox;
    profile->setObjectName("machineProfileName");
    profile->setEditable(true);
    profile->addItem(copy.machine.printer["name"].toString("Custom research printer"));
    profile->setMinimumHeight(42);
    bar->addWidget(profile, 1);
    auto save = new QToolButton;
    save->setObjectName("saveMachineProfile");
    save->setIcon(dialog.style()->standardIcon(QStyle::SP_DialogSaveButton));
    save->setToolTip("Save this machine profile…");
    bar->addWidget(save);
    auto load = new QToolButton;
    load->setIcon(dialog.style()->standardIcon(QStyle::SP_DialogOpenButton));
    load->setToolTip("Load a machine profile…");
    bar->addWidget(load);
    auto searchButton = new QToolButton;
    searchButton->setObjectName("machineSearchButton");
    searchButton->setIcon(dialog.style()->standardIcon(QStyle::SP_FileDialogContentsView));
    searchButton->setToolTip("Search printer settings");
    bar->addWidget(searchButton);
    auto advanced = new AdvancedSwitch;
    advanced->setObjectName("machineAdvanced");
    advanced->setChecked(copy.machine.printer["advanced"].toBool());
    bar->addWidget(advanced);
    auto search = new QLineEdit;
    search->setObjectName("machineSearch");
    search->setPlaceholderText("Search printer settings…");
    search->hide();
    root->addWidget(search);
    auto tabs = new QTabWidget;
    tabs->setObjectName("machineSettingsTabs");
    root->addWidget(tabs, 1);
    std::vector<Row> rows;
    std::vector<std::function<void()>> commits;
    std::vector<QGroupBox *> groups;
    auto page = [&](const QString &title) {
        auto scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        auto content = new QWidget;
        content->setObjectName("page");
        scroll->setWidget(content);
        auto layout = new QVBoxLayout(content);
        layout->setContentsMargins(8, 10, 18, 20);
        layout->setSpacing(16);
        tabs->addTab(scroll, title);
        return layout;
    };
    auto section = [&](QVBoxLayout *pageLayout, const QString &title) {
        auto group = new QGroupBox(title);
        groups.push_back(group);
        pageLayout->addWidget(group);
        auto form = new QFormLayout(group);
        form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        form->setFormAlignment(Qt::AlignTop | Qt::AlignLeft);
        form->setHorizontalSpacing(30);
        form->setVerticalSpacing(10);
        return form;
    };
    auto row = [&](QFormLayout *form, const QString &label, QWidget *field, bool adv = false) {
        auto text = new QLabel(label);
        text->setFixedWidth(255);
        text->setWordWrap(true);
        text->setBuddy(field);
        form->addRow(text, field);
        rows.push_back({text, field, label, adv});
        field->setAccessibleName(label);
        if (auto button = qobject_cast<QPushButton *>(field)) {
            button->setAutoDefault(false);
            button->setMaximumWidth(360);
        }
        if (qobject_cast<QComboBox *>(field))
            field->setMaximumWidth(440);
    };
    auto setting = [&](QFormLayout *form, const char *key, const QString &label, double value, double min,
                       double max, const QString &unit, bool adv = true) {
        auto field = number(value, min, max, unit, key);
        row(form, label, field, adv);
        commits.push_back([&, field, key] { copy.machine.printer[key] = field->value(); });
        return field;
    };
    auto toggle = [&](QFormLayout *form, const char *key, const QString &label, bool value, bool adv = true) {
        auto field = new PrinterCheckBox;
        field->setObjectName(key);
        field->setChecked(value);
        row(form, label, field, adv);
        commits.push_back([&, field, key] { copy.machine.printer[key] = field->isChecked(); });
        return field;
    };
    auto basic = page("Basic information");
    auto printable = section(basic, "Printable space");
    auto bedButton = new QPushButton("Set…");
    bedButton->setFixedWidth(170);
    bedButton->setObjectName("editBedShape");
    row(printable, "Printable area", bedButton, true);
    bedButton->setFixedWidth(170);
    auto bedEnabled = toggle(printable, "bedEnabled", "Enforce printable area",
                             copy.machine.printer["bedEnabled"].toBool(), true);
    auto exclusions = new QLineEdit;
    exclusions->setObjectName("bedExclusions");
    exclusions->setPlaceholderText("0x0, 18x0, 18x28, 0x28; separate polygons with ;");
    QStringList excluded;
    for (const auto &v : copy.machine.printer["bedExclusions"].toArray())
        excluded << polygon_text(v.toArray());
    exclusions->setText(excluded.join("; "));
    row(printable, "Excluded bed area", exclusions, true);
    auto printableHeight =
        setting(printable, "printableHeight", "Printable height",
                printer_number(copy.machine.printer, "printableHeight", copy.machine.maxXYZ.z()), .1, 10000,
                "mm", false);
    setting(printable, "preferredX", "Preferred object position X",
            printer_number(copy.machine.printer, "preferredX", .5), 0, 1, {}, true);
    setting(printable, "preferredY", "Preferred object position Y",
            printer_number(copy.machine.printer, "preferredY", .5), 0, 1, {}, true);
    auto firmware = section(basic, "Machine and controller");
    auto controller = new QComboBox;
    controller->setObjectName("machineController");
    controller->addItems({"Duet / RepRapFirmware", "Klipper", "Generic / Marlin XYZ"});
    controller->setCurrentIndex(int(copy.machine.controller));
    row(firmware, "G-code type", controller);
    auto layout = new QComboBox;
    layout->setObjectName("machineLayout");
    layout->addItems({"Rotary bed · A / C", "Tilting head · A / B", "Standard XYZ · fixed nozzle"});
    layout->setCurrentIndex(int(copy.machine.layout));
    row(firmware, "Kinematics", layout);
    auto relative =
        toggle(firmware, "relativeE", "Use relative E distances", relative_extrusion(copy.machine));
    auto firmwareRetract = new PrinterCheckBox;
    firmwareRetract->setEnabled(false);
    firmwareRetract->setToolTip("Retraction is emitted as explicit E and hop movements so collision checking "
                                "and replay see the complete trajectory.");
    row(firmware, "Use firmware retraction", firmwareRetract, true);
    for (auto value : {std::make_tuple("loadSeconds", "Filament load time", 0.0),
                       {"unloadSeconds", "Filament unload time", 0.0},
                       {"extruderChangeSeconds", "Extruder change time", 0.0},
                       {"hotendChangeSeconds", "Hotend change time", 0.0}})
        setting(firmware, std::get<0>(value), std::get<1>(value),
                printer_number(copy.machine.printer, std::get<0>(value), std::get<2>(value)), 0, 3600, "s");
    auto clearanceSection = section(basic, "Extruder clearance · profile reference");
    for (auto value : {std::make_tuple("clearanceRadius", "Maximum radius", 68.0),
                       {"rodDistance", "Distance to rod", 33.0},
                       {"rodHeight", "Height to rod", 34.0},
                       {"lidHeight", "Height to lid", 90.0}})
        setting(clearanceSection, std::get<0>(value), std::get<1>(value),
                printer_number(copy.machine.printer, std::get<0>(value), std::get<2>(value)), 0, 10000, "mm");
    auto accessories = section(basic, "Accessories");
    toggle(accessories, "airFiltration", "Air filtration accessory",
           copy.machine.printer["airFiltration"].toBool());
    basic->addStretch();
    auto gcodePage = page("Machine G-code");
    auto help =
        new QLabel("Absolute calibrated carriage coordinates · millimetres. G0/G1 travel, G4 dwell and M400 "
                   "are modeled and collision checked. Homing, extrusion, expressions and arbitrary macros "
                   "need a controller recipe. Start/layer/object hooks return to the resume pose.");
    help->setObjectName("hint");
    help->setWordWrap(true);
    gcodePage->addWidget(help);
    QMap<QString, QPlainTextEdit *> scripts;
    const std::vector<std::pair<const char *, const char *>> scriptNames{
        {"start", "Machine start G-code"},
        {"end", "Machine end G-code"},
        {"betweenObjects", "Between objects G-code"},
        {"layerChange", "Layer change G-code"},
        {"timelapse", "Timelapse G-code"},
        {"filamentChange", "Filament change G-code"},
        {"pause", "Pause G-code · saved template"},
        {"custom", "Custom G-code template"}};
    for (const auto &entry : scriptNames) {
        auto group = new QGroupBox(entry.second);
        auto v = new QVBoxLayout(group);
        auto field = new QPlainTextEdit(copy.machine.printer["gcode"].toObject()[entry.first].toString());
        field->setObjectName(QString("gcode_") + entry.first);
        field->setMinimumHeight(160);
        field->setFont(QFont("Consolas", 11));
        field->setTabStopDistance(28);
        v->addWidget(field);
        gcodePage->addWidget(group);
        scripts[entry.first] = field;
    }
    auto timelapse = new PrinterCheckBox("Run timelapse hook at layer changes");
    timelapse->setChecked(copy.machine.printer["timelapseEnabled"].toBool());
    gcodePage->insertWidget(1, timelapse);
    auto motion = page("Motion capability");
    QString previousSection;
    QFormLayout *motionForm = nullptr;
    for (const auto &option : motion_options()) {
        if (previousSection != option.section) {
            previousSection = option.section;
            motionForm = section(motion, previousSection);
        }
        double fallback = option.initial;
        if (QString(option.key) == "speedX" || QString(option.key) == "speedY")
            fallback = copy.machine.linearSpeed;
        if (QString(option.key).startsWith("accel") && QString(option.key) != "accelE" &&
            QString(option.key) != "accelRetract")
            fallback = copy.machine.linearAccel;
        setting(motionForm, option.key, option.label,
                printer_number(copy.machine.printer, option.key, fallback), option.minimum, option.maximum,
                option.unit, false);
    }
    motion->addStretch();
    auto extruder = page("Extruder");
    auto toolForm = section(extruder, "Installed extruder / nozzle");
    auto tools = new QComboBox;
    tools->setObjectName("machineTool");
    if (copy.plateEnabled)
        for (const auto &tool : copy.tools)
            tools->addItem(tool.name + " · " + tool.id, tool.id);
    else
        tools->addItem("Primary tool");
    row(toolForm, "Physical tool", tools, true);
    auto manageTools = new QPushButton("Tools, materials and change recipes…");
    manageTools->setObjectName("manageMachineTools");
    row(toolForm, "Plate / tools", manageTools, true);
    auto type = new QComboBox;
    type->addItems({"Direct drive", "Bowden"});
    row(toolForm, "Type", type, true);
    auto diameter = number(copy.process.nozzleDiameter, .05, 5, "mm", "machineNozzleDiameter");
    row(toolForm, "Nozzle diameter", diameter, true);
    auto limits = section(extruder, "Local bead dimensions");
    auto automatic = new PrinterCheckBox("Auto-fill ½× to 2× when diameter changes");
    automatic->setObjectName("machineAutoBeadLimits");
    row(limits, "Automatic limits", automatic, true);
    auto minH = number(.05, .001, 10, "mm", "machineMinBeadHeight"),
         maxH = number(.3, .001, 10, "mm", "machineMaxBeadHeight"),
         minW = number(.32, .001, 10, "mm", "machineMinBeadWidth"),
         maxW = number(.8, .001, 10, "mm", "machineMaxBeadWidth");
    row(limits, "Minimum local bead height", minH, true);
    row(limits, "Maximum local bead height", maxH, true);
    row(limits, "Minimum local bead width", minW, true);
    row(limits, "Maximum local bead width", maxW, true);
    auto position = section(extruder, "Position · carriage to nozzle TCP");
    auto tcpX = number(0, -1000, 1000, "mm", "extruderOffsetX"),
         tcpY = number(0, -1000, 1000, "mm", "extruderOffsetY"),
         tcpZ = number(0, -1000, 1000, "mm", "extruderOffsetZ");
    row(position, "Extruder offset X", tcpX, true);
    row(position, "Extruder offset Y", tcpY, true);
    row(position, "Extruder offset Z", tcpZ, true);
    auto retraction = section(extruder, "Retraction");
    auto retractEnabled = new PrinterCheckBox;
    retractEnabled->setObjectName("printerRetractionEnabled");
    row(retraction, "Enable planned retraction", retractEnabled);
    QMap<QString, QDoubleSpinBox *> extruderFields;
    for (const auto &option : extruder_options()) {
        auto field = number(option.initial, option.minimum, option.maximum, option.unit, option.key);
        extruderFields[option.key] = field;
        row(QString(option.section) == "Basic information" ? toolForm : retraction, option.label, field,
            option.advanced);
    }
    auto layerRetract = new PrinterCheckBox, wipe = new PrinterCheckBox;
    row(retraction, "Retract at layer change", layerRetract, true);
    row(retraction, "Wipe while retracting", wipe, true);
    extruder->addStretch();
    int toolIndex = 0;
    bool loading = false;
    auto storeTool = [&] {
        if (loading)
            return;
        QJsonObject ex;
        for (auto it = extruderFields.begin(); it != extruderFields.end(); ++it)
            ex[it.key()] = it.value()->value();
        ex["enabled"] = retractEnabled->isChecked();
        ex["type"] = type->currentText();
        ex["retractLayerChange"] = layerRetract->isChecked();
        ex["wipe"] = wipe->isChecked();
        if (copy.plateEnabled && toolIndex < int(copy.tools.size())) {
            auto &tool = copy.tools[toolIndex];
            tool.extruderSettings = ex;
            bool changed = std::abs(tool.nozzle.diameter - diameter->value()) > 1e-8;
            tool.nozzle.diameter = diameter->value();
            tool.nozzle.minHeight = minH->value();
            tool.nozzle.maxHeight = maxH->value();
            tool.nozzle.minWidth = minW->value();
            tool.nozzle.maxWidth = maxW->value();
            tool.nozzle.autoBeadLimits = automatic->isChecked();
            auto &base = tool.useDummy ? tool.dummy : tool.assembly;
            Vec3 tip(tcpX->value(), tcpY->value(), tcpZ->value());
            auto installed = installed_assembly(tool);
            Vec3 difference = tip - installed.mount * installed.tcp;
            if (difference.norm() > 1e-6) {
                base.mount.translation() += difference;
                tool.nozzle.calibrated = false;
            }
            if (changed) {
                tool.nozzle.calibrated = false;
                if (tool.useDummy) {
                    auto mount = tool.dummy.mount;
                    tool.dummy = make_dummy_toolhead(tool.nozzle.diameter);
                    tool.dummy.mount = mount;
                }
            }
        } else {
            copy.machine.printer["extruder"] = ex;
            copy.process.nozzleDiameter = diameter->value();
        }
    };
    auto loadTool = [&] {
        loading = true;
        QJsonObject ex;
        Vec3 tip = copy.active_toolhead().mount * copy.active_toolhead().tcp;
        if (copy.plateEnabled && toolIndex < int(copy.tools.size())) {
            const auto &tool = copy.tools[toolIndex];
            ex = tool.extruderSettings;
            diameter->setValue(tool.nozzle.diameter);
            automatic->setChecked(tool.nozzle.autoBeadLimits);
            minH->setValue(tool.nozzle.minHeight);
            maxH->setValue(tool.nozzle.maxHeight);
            minW->setValue(tool.nozzle.minWidth);
            maxW->setValue(tool.nozzle.maxWidth);
            auto assembly = installed_assembly(tool);
            tip = assembly.mount * assembly.tcp;
        } else {
            ex = copy.machine.printer["extruder"].toObject();
            diameter->setValue(copy.process.nozzleDiameter);
            automatic->setChecked(true);
            minH->setValue(diameter->value() * .5);
            minW->setValue(diameter->value() * .5);
            maxH->setValue(diameter->value() * 2);
            maxW->setValue(diameter->value() * 2);
        }
        tcpX->setValue(tip.x());
        tcpY->setValue(tip.y());
        tcpZ->setValue(tip.z());
        for (const auto &option : extruder_options())
            extruderFields[option.key]->setValue(printer_number(ex, option.key, option.initial));
        retractEnabled->setChecked(ex["enabled"].toBool());
        type->setCurrentIndex(ex["type"].toString() == "Bowden" ? 1 : 0);
        layerRetract->setChecked(ex["retractLayerChange"].toBool(true));
        wipe->setChecked(ex["wipe"].toBool());
        loading = false;
    };
    QObject::connect(diameter, qOverload<double>(&QDoubleSpinBox::valueChanged), &dialog, [&](double value) {
        if (loading || !automatic->isChecked())
            return;
        minH->setValue(value * .5);
        minW->setValue(value * .5);
        maxH->setValue(value * 2);
        maxW->setValue(value * 2);
    });
    QObject::connect(automatic, &QCheckBox::toggled, &dialog, [&](bool checked) {
        if (loading || !checked)
            return;
        minH->setValue(diameter->value() * .5);
        minW->setValue(diameter->value() * .5);
        maxH->setValue(diameter->value() * 2);
        maxW->setValue(diameter->value() * 2);
    });
    QObject::connect(tools, &QComboBox::currentIndexChanged, &dialog, [&](int index) {
        storeTool();
        toolIndex = std::max(0, index);
        loadTool();
    });
    auto notesPage = page("Notes");
    auto notes = new QPlainTextEdit(copy.machine.printer["notes"].toString());
    notes->setObjectName("machineNotes");
    notes->setPlaceholderText("Calibration notes, machine changes and qualifications…");
    notesPage->addWidget(notes, 1);
    auto axesPage = page("Axes / STEP");
    auto axesForm = section(axesPage, "Machine links and motion envelope");
    QMap<QString, QLineEdit *> vectors;
    for (auto value : {std::make_pair("XYZ minimum", copy.machine.minXYZ),
                       {"XYZ maximum", copy.machine.maxXYZ},
                       {"Bed pivot", copy.machine.bedPivot},
                       {"Head A pivot", copy.machine.headPivotA},
                       {"Head B pivot", copy.machine.headPivotB}}) {
        auto field = new QLineEdit(
            QString("%1, %2, %3").arg(value.second.x()).arg(value.second.y()).arg(value.second.z()));
        vectors[value.first] = field;
        row(axesForm, QString(value.first) + " · mm", field);
    }
    auto minA = number(copy.machine.minA * 180 / pi, -7200, 7200, "°", "machineMinA"),
         maxA = number(copy.machine.maxA * 180 / pi, -7200, 7200, "°", "machineMaxA"),
         minB = number(copy.machine.minB * 180 / pi, -7200, 7200, "°", "machineMinB"),
         maxB = number(copy.machine.maxB * 180 / pi, -7200, 7200, "°", "machineMaxB");
    row(axesForm, "A minimum", minA);
    row(axesForm, "A maximum", maxA);
    row(axesForm, "B / C minimum", minB);
    row(axesForm, "B / C maximum", maxB);
    auto rotarySpeed = number(copy.machine.angularSpeed * 180 / pi, .01, 3600, "°/s", "rotarySpeed"),
         rotaryAccel =
             number(copy.machine.angularAccel * 180 / pi, .01, 100000, "°/s²", "rotaryAcceleration");
    row(axesForm, "Maximum rotary speed", rotarySpeed);
    row(axesForm, "Maximum rotary acceleration", rotaryAccel);
    auto extension = new PrinterCheckBox;
    extension->setChecked(copy.machine.klipperExtension);
    row(axesForm, "Klipper rotary-aware flow extension", extension);
    auto assemblyForm = section(axesPage, "STEP toolhead and calibration");
    auto assemblyTools = new QComboBox;
    assemblyTools->setObjectName("calibrationTool");
    auto refreshAssemblyTools = [&] {
        auto current = assemblyTools->currentData().toString();
        assemblyTools->clear();
        if (copy.plateEnabled)
            for (const auto &tool : copy.tools)
                assemblyTools->addItem(tool.name + " · " + (tool.useDummy ? "Dummy" : "STEP"), tool.id);
        else
            assemblyTools->addItem("Primary tool");
        int index = assemblyTools->findData(current);
        if (index >= 0)
            assemblyTools->setCurrentIndex(index);
    };
    row(assemblyForm, "Physical assembly", assemblyTools);
    auto import = new QPushButton("Import toolhead STEP…"),
         calibrate = new QPushButton("Components, TCP and pose preview…");
    row(assemblyForm, "Source geometry", import);
    row(assemblyForm, "Link attachments and nozzle contact", calibrate);
    auto dummy = new PrinterCheckBox;
    row(assemblyForm, "Generate with dummy toolhead", dummy);
    auto assemblyHint = new QLabel;
    assemblyHint->setWordWrap(true);
    assemblyHint->setObjectName("hint");
    axesPage->addWidget(assemblyHint);
    axesPage->addStretch();
    auto selectedTool = [&]() -> ToolDefinition * {
        auto id = assemblyTools->currentData().toString();
        for (auto &tool : copy.tools)
            if (tool.id == id)
                return &tool;
        return nullptr;
    };
    auto refreshGeometry = [&] {
        QSignalBlocker block(dummy);
        auto tool = selectedTool();
        bool synthetic = tool ? tool->useDummy : copy.useDummyToolhead;
        dummy->setChecked(synthetic);
        auto assembly = tool ? installed_assembly(*tool) : copy.active_toolhead();
        assemblyHint->setText(
            QString("%1 · %2 components · source units: %3\nGeometry and calibration correspond to the "
                    "installed nozzle. Diameter edits never resize imported CAD.")
                .arg(synthetic ? "Dimensioned dummy geometry" : assembly.sourceName)
                .arg(assembly.components.size())
                .arg(assembly.sourceUnits.isEmpty() ? "mm" : assembly.sourceUnits));
    };
    QObject::connect(assemblyTools, &QComboBox::currentIndexChanged, &dialog,
                     [&](int) { refreshGeometry(); });
    QObject::connect(dummy, &QCheckBox::toggled, &dialog, [&](bool value) {
        storeTool();
        if (auto tool = selectedTool()) {
            tool->useDummy = value;
            if (value && tool->dummy.components.empty())
                tool->dummy = make_dummy_toolhead(tool->nozzle.diameter);
            tool->nozzle.calibrated = false;
        } else {
            copy.useDummyToolhead = value;
            if (value && copy.dummyToolhead.components.empty())
                copy.dummyToolhead = make_dummy_toolhead(copy.process.nozzleDiameter);
        }
        refreshGeometry();
    });
    QObject::connect(import, &QPushButton::clicked, &dialog, [&] {
        storeTool();
        auto file = QFileDialog::getOpenFileName(&dialog, "Toolhead STEP", {}, "STEP (*.step *.stp)");
        if (file.isEmpty())
            return;
        ToolheadAssembly assembly;
        if (!import_step_dialog(&dialog, file, assembly, true))
            return;
        if (auto tool = selectedTool()) {
            tool->assembly = assembly;
            tool->useDummy = false;
            tool->nozzle.calibrated = false;
        } else {
            copy.toolhead = assembly;
            copy.useDummyToolhead = false;
        }
        refreshGeometry();
    });
    std::function<void()> flush, synchronizeCalibration;
    QObject::connect(calibrate, &QPushButton::clicked, &dialog, [&] {
        try {
            flush();
            if (edit_toolhead(copy, &dialog, assemblyTools->currentData().toString())) {
                synchronizeCalibration();
                loadTool();
                refreshGeometry();
            }
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Calibration", error.what());
        }
    });
    QObject::connect(manageTools, &QPushButton::clicked, &dialog, [&] {
        storeTool();
        if (edit_plate(copy, &dialog)) {
            loading = true;
            tools->clear();
            for (const auto &tool : copy.tools)
                tools->addItem(tool.name + " · " + tool.id, tool.id);
            toolIndex = 0;
            loading = false;
            loadTool();
            refreshAssemblyTools();
            refreshGeometry();
        }
    });
    auto collisionPage = page("Collision");
    auto collisionForm = section(collisionPage, "Continuous clearance checking");
    auto clearance = number(copy.machine.clearance, 0, 100, "mm", "machineClearance"),
         chord = number(copy.machine.chordTolerance, .0001, 2, "mm", "machineChordTolerance"),
         angle = number(copy.machine.angularTolerance * 180 / pi, .001, 20, "°", "machineAngularTolerance");
    row(collisionForm, "Required non-contact clearance", clearance);
    row(collisionForm, "Motion chord accuracy", chord);
    row(collisionForm, "Angular interval accuracy", angle);
    auto bodies = new PrinterCheckBox;
    bodies->setChecked(copy.checkWorkpieceBodies);
    row(collisionForm, "Check rotating workpiece against machine bodies", bodies);
    auto collisionHint =
        new QLabel("Active and parked tools, docks, fixtures, completed objects and accumulated deposition "
                   "remain in the collision scene. Only the active nozzle gets local bead-contact "
                   "allowances. Unresolved intervals block complete-job export.");
    collisionHint->setWordWrap(true);
    collisionHint->setObjectName("hint");
    collisionPage->addWidget(collisionHint);
    collisionPage->addStretch();
    auto status = new QLabel("Edits require regeneration and fresh motion validation.");
    status->setObjectName("hint");
    root->addWidget(status);
    auto buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Save)->setDefault(true);
    root->addWidget(buttons);
    auto readVector = [&](const char *key) {
        auto tokens = vectors[key]->text().split(QRegularExpression("[,;\\s]+"), Qt::SkipEmptyParts);
        if (tokens.size() != 3)
            throw std::runtime_error("Enter three coordinates separated by commas");
        Vec3 v;
        for (int i = 0; i < 3; ++i) {
            bool ok;
            v[i] = tokens[i].toDouble(&ok);
            if (!ok || !std::isfinite(v[i]))
                throw std::runtime_error("Invalid machine coordinate");
        }
        return v;
    };
    auto commit = [&] {
        storeTool();
        for (auto &f : commits)
            f();
        copy.machine.printer["name"] = profile->currentText();
        copy.machine.printer["notes"] = notes->toPlainText();
        copy.machine.printer["advanced"] = advanced->isChecked();
        QJsonObject code;
        for (auto it = scripts.begin(); it != scripts.end(); ++it)
            code[it.key()] = it.value()->toPlainText();
        copy.machine.printer["gcode"] = code;
        copy.machine.printer["timelapseEnabled"] = timelapse->isChecked();
        QJsonArray excluded;
        for (const auto &line : exclusions->text().split(';', Qt::SkipEmptyParts))
            excluded.append(parse_polygon(line));
        copy.machine.printer["bedExclusions"] = excluded;
        copy.machine.controller = Controller(controller->currentIndex());
        copy.machine.layout = Layout(layout->currentIndex());
        copy.machine.klipperExtension = extension->isChecked();
        copy.machine.minXYZ = readVector("XYZ minimum");
        copy.machine.maxXYZ = readVector("XYZ maximum");
        copy.machine.bedPivot = readVector("Bed pivot");
        copy.machine.headPivotA = readVector("Head A pivot");
        copy.machine.headPivotB = readVector("Head B pivot");
        copy.machine.minA = minA->value() * pi / 180;
        copy.machine.maxA = maxA->value() * pi / 180;
        copy.machine.minB = minB->value() * pi / 180;
        copy.machine.maxB = maxB->value() * pi / 180;
        copy.machine.angularSpeed = rotarySpeed->value() * pi / 180;
        copy.machine.angularAccel = rotaryAccel->value() * pi / 180;
        copy.machine.clearance = clearance->value();
        copy.machine.chordTolerance = chord->value();
        copy.machine.angularTolerance = angle->value() * pi / 180;
        copy.checkWorkpieceBodies = bodies->isChecked();
        copy.machine.maxXYZ.z() = printableHeight->value();
        copy.machine.linearSpeed =
            std::max({copy.machine.printer["speedX"].toDouble(), copy.machine.printer["speedY"].toDouble(),
                      copy.machine.printer["speedZ"].toDouble()});
        copy.machine.linearAccel =
            std::max({copy.machine.printer["accelX"].toDouble(), copy.machine.printer["accelY"].toDouble(),
                      copy.machine.printer["accelZ"].toDouble()});
        if ((copy.machine.maxXYZ - copy.machine.minXYZ).minCoeff() <= 0 ||
            copy.machine.maxA <= copy.machine.minA || copy.machine.maxB <= copy.machine.minB)
            throw std::runtime_error("Axis maximum must exceed minimum");
        if (copy.machine.controller == Controller::GenericMarlin && copy.machine.rotary_axes())
            throw std::runtime_error("Generic / Marlin is available for XYZ printers only");
        validate_printer_settings(copy.machine);
        for (const auto &tool : copy.tools)
            validate_extruder_settings(tool.extruderSettings);
        copy.invalidate();
        if (copy.plateEnabled)
            synchronize_plate_preview(copy);
    };
    flush = commit;
    synchronizeCalibration = [&] {
        for (auto value : {std::make_pair("XYZ minimum", copy.machine.minXYZ),
                           {"XYZ maximum", copy.machine.maxXYZ},
                           {"Bed pivot", copy.machine.bedPivot},
                           {"Head A pivot", copy.machine.headPivotA},
                           {"Head B pivot", copy.machine.headPivotB}})
            vectors[value.first]->setText(
                QString("%1, %2, %3").arg(value.second.x()).arg(value.second.y()).arg(value.second.z()));
        clearance->setValue(copy.machine.clearance);
    };
    QObject::connect(bedButton, &QPushButton::clicked, &dialog, [&] {
        copy.machine.printer["printableHeight"] = printableHeight->value();
        if (edit_bed(copy, &dialog))
            bedEnabled->setChecked(true);
    });
    auto filter = [&] {
        bool adv = advanced->isChecked() || !search->text().trimmed().isEmpty();
        for (auto &r : rows) {
            bool visible = (!r.advanced || adv) &&
                           (search->text().isEmpty() || r.text.contains(search->text(), Qt::CaseInsensitive));
            r.label->setVisible(visible);
            r.field->setVisible(visible);
        }
        for (auto group : groups) {
            bool any = false;
            for (auto &r : rows)
                if (group->isAncestorOf(r.field) && !r.field->isHidden())
                    any = true;
            group->setVisible(any);
        }
        for (int i : {1, 4, 5, 6})
            tabs->setTabVisible(i, adv);
    };
    QObject::connect(advanced, &QCheckBox::toggled, &dialog, [&](bool) { filter(); });
    QObject::connect(search, &QLineEdit::textChanged, &dialog, [&](const QString &query) {
        filter();
        if (query.trimmed().isEmpty())
            return;
        for (const auto &r : rows)
            if (!r.field->isHidden()) {
                for (int i = 0; i < tabs->count(); ++i)
                    if (tabs->widget(i)->isAncestorOf(r.field)) {
                        tabs->setCurrentIndex(i);
                        return;
                    }
            }
    });
    QObject::connect(searchButton, &QToolButton::clicked, &dialog, [&] {
        search->setVisible(!search->isVisible());
        if (search->isVisible())
            search->setFocus();
        else {
            search->clear();
            filter();
        }
    });
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, [&] {
        try {
            commit();
            project = std::move(copy);
            dialog.accept();
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Printer settings", error.what());
        }
    });
    QObject::connect(save, &QToolButton::clicked, &dialog, [&] {
        try {
            commit();
            auto file = QFileDialog::getSaveFileName(&dialog, "Save machine profile", {},
                                                     "FiveAxisLab machine (*.5machine)");
            if (file.isEmpty())
                return;
            Project saved = copy;
            saved.objects.clear();
            saved.part = {};
            saved.volume = {};
            saved.path.clear();
            saved.events.clear();
            saved.invalidate();
            save_project(saved, file);
            status->setText("Machine profile saved: " + QFileInfo(file).fileName());
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Save profile", error.what());
        }
    });
    QObject::connect(load, &QToolButton::clicked, &dialog, [&] {
        auto file = QFileDialog::getOpenFileName(&dialog, "Load machine profile", {},
                                                 "FiveAxisLab machine (*.5machine)");
        if (file.isEmpty())
            return;
        try {
            auto loaded = load_project(file);
            Project changed = project;
            changed.machine = loaded.machine;
            changed.fixtures = loaded.fixtures;
            changed.toolhead = loaded.toolhead;
            changed.dummyToolhead = loaded.dummyToolhead;
            changed.useDummyToolhead = loaded.useDummyToolhead;
            if (loaded.plateEnabled) {
                if (!changed.plateEnabled)
                    enable_plate(changed);
                changed.tools = loaded.tools;
                changed.carriageAssembly = loaded.carriageAssembly;
                for (const auto &m : loaded.materials)
                    if (!find_material(changed, m.id))
                        changed.materials.push_back(m);
            }
            changed.invalidate();
            if (edit_machine(changed, &dialog)) {
                project = std::move(changed);
                dialog.accept();
            }
        } catch (const std::exception &error) {
            QMessageBox::warning(&dialog, "Load profile", error.what());
        }
    });
    loadTool();
    refreshAssemblyTools();
    refreshGeometry();
    filter();
    return dialog.exec() == QDialog::Accepted;
}
