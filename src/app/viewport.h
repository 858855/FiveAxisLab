#pragma once
#include "core/types.h"
#include <QLabel>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <map>
#include <set>
class Viewport : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
  public:
    explicit Viewport(QWidget *parent = nullptr);
    ~Viewport() override;
    void set_project(const fiveaxis::Project *p);
    void set_time(double t);
    void set_stress(bool v);
    void set_follow(bool v) {
        follow = v;
        update();
    }
    void set_transparency(bool v) {
        transparent = v;
        update();
    }
    void set_tool_pick(bool v) {
        pickTool = v;
    }
    void set_selected(const std::set<int> &fixed, const std::set<int> &loaded);
    void fit();
  Q_SIGNALS:
    void face_picked(int triangle);
    void tool_picked(int component, fiveaxis::Vec3 point);

  protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int, int) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void wheelEvent(QWheelEvent *) override;

  private:
    const fiveaxis::Project *project = nullptr;
    fiveaxis::SimulationState state;
    QOpenGLShaderProgram shader;
    double time = 0, yaw = -55, pitch = 30, distance = 110;
    fiveaxis::Vec3 target = fiveaxis::Vec3(0, 0, 8);
    QPoint last, pressed;
    bool follow = false, transparent = true, pickTool = false;
    bool stressVisible = false;
    bool validationCurrent = false;
    double fieldMin = 0, fieldMax = 1;
    std::set<int> fixed, loaded;
    QMatrix4x4 view_projection() const;
    struct RenderMesh {
        QOpenGLBuffer buffer;
        int count = 0;
    };
    std::map<const fiveaxis::Mesh *, std::unique_ptr<RenderMesh>> cache;
    fiveaxis::Mesh beads, unitBox, fixedMesh, loadedMesh, marker;
    std::vector<int> beadEnds;
    void draw_mesh(const fiveaxis::Mesh &, const fiveaxis::Transform &, QVector4D, int triangles = -1);
    void pick(QPoint);
    QLabel *hud;
};
