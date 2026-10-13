#pragma once
#include "core/types.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

class AxisPreview : public QGroupBox {
    Q_OBJECT
  public:
    explicit AxisPreview(QWidget *parent=nullptr);
    void set_project(const fiveaxis::Project *project,const fiveaxis::JointState &recorded);
    void follow_motion(const fiveaxis::JointState &recorded);
    bool overriding() const {return source->currentIndex()!=0;}
    void pause();
    void toggle_animation();
  Q_SIGNALS:
    void pose_changed(fiveaxis::JointState joint,bool overrideMotion);
  private:
    const fiveaxis::Project *project=nullptr;
    fiveaxis::JointState recorded,displayed;
    fiveaxis::Vec3 anchor=fiveaxis::Vec3::Zero();
    QComboBox *source;
    QDoubleSpinBox *a,*b;
    QSlider *sliderA,*sliderB;
    QLabel *labelA,*labelB,*status;
    QPushButton *animate;
    QTimer timer;
    QElapsedTimer elapsed;
    double phase=0;
    bool updating=false;
    void set_angles(double first,double second);
    void apply_pose();
    void change_source(int index);
    void tick();
};
