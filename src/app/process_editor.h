#pragma once
#include "core/types.h"
#include <QCheckBox>
#include <QLineEdit>
#include <QMap>
#include <QTabWidget>
#include <QWidget>

// The inline editor and the larger dialog share the same bindings and validation.
class ProcessEditor : public QWidget {
    Q_OBJECT
  public:
    explicit ProcessEditor(QWidget *parent=nullptr);
    void set_settings(const fiveaxis::ProcessSettings &settings);
    void set_layout(fiveaxis::Layout layout);
    fiveaxis::ProcessSettings settings() const;
    bool valid() const { return error.isEmpty(); }
    QString validation_error() const { return error; }
    QTabWidget *tabs() const { return pages; }
  Q_SIGNALS:
    void edited();
  private:
    struct Binding { QWidget *input=nullptr,*row=nullptr; double scale=1; bool advanced=false; QString label; };
    QMap<QString,Binding> bindings;
    QTabWidget *pages;
    QCheckBox *advanced;
    QLineEdit *search;
    fiveaxis::ProcessSettings base;
    QString error;
    bool updating=false;
    void changed();
    void filter();
};
