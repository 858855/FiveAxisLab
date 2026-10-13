#include "main_window.h"
#include "core/geometry.h"
#include "core/process.h"
#include <QDialog>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QPushButton>
#include <QVBoxLayout>
using namespace fiveaxis;
void MainWindow::process_settings() {
    if(worker.isRunning()) return;
    QDialog dialog(this); dialog.setObjectName("processSettingsDialog"); dialog.setWindowTitle("Print process · Orca options"); dialog.resize(640,850);
    auto outer=new QVBoxLayout(&dialog); auto editor=new ProcessEditor; editor->set_settings(project.process);editor->set_layout(project.machine.layout); outer->addWidget(editor,1);
    auto error=new QLabel; error->setWordWrap(true); error->setStyleSheet("color:#bc3030;"); outer->addWidget(error);
    auto buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel); outer->addWidget(buttons);
    connect(editor,&ProcessEditor::edited,&dialog,[&]{error->setText(editor->validation_error());buttons->button(QDialogButtonBox::Ok)->setEnabled(editor->valid());});
    connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
    connect(buttons,&QDialogButtonBox::accepted,&dialog,[&] {
        if(!editor->valid()) return; auto next=editor->settings();
        if(next.nozzleDiameter!=project.process.nozzleDiameter) project.dummyToolhead=make_dummy_toolhead(next.nozzleDiameter);
        project.process=next; project.invalidate(); replaying=false; processEditor->set_settings(next); refresh(); dialog.accept();
    });
    dialog.exec();
}
void MainWindow::import_orca_settings() {
    if(worker.isRunning()) return;
    auto file=QFileDialog::getOpenFileName(this,"Import Orca or FiveAxisLab process preset",{},"Process JSON (*.json)"); if(file.isEmpty()) return;
    try {
        QFile input(file); if(!input.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot open process preset");
        QJsonParseError error; auto doc=QJsonDocument::fromJson(input.readAll(),&error);
        if(error.error!=QJsonParseError::NoError || !doc.isObject()) throw std::runtime_error("Invalid process JSON");
        QStringList notes; auto next=doc.object().contains("layerHeight")?read_process(doc.object()):import_orca_process(doc.object(),project.process,notes);
        if(next.nozzleDiameter!=project.process.nozzleDiameter) project.dummyToolhead=make_dummy_toolhead(next.nozzleDiameter);
        project.process=next; project.invalidate(); replaying=false; processEditor->set_settings(next); refresh();
        log->append("Process imported. Slice again to apply it."); for(const auto &note:notes) log->append(note);
    } catch(const std::exception &e) {show_error(e.what());}
}
