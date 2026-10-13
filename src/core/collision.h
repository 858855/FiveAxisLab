#pragma once
#include "types.h"
#include <memory>
namespace fiveaxis {
class CollisionScene {
  public:
    explicit CollisionScene(const Project &project);
    ~CollisionScene();
    CollisionScene(const CollisionScene &) = delete;
    CollisionScene &operator=(const CollisionScene &) = delete;
    CollisionEvent check_pose(const JointState &q, bool extruding, const Vec3 &activePoint,
                              double margin = -1) const;
    bool check_interval(const JointState &a, const JointState &b, bool extruding, const Vec3 &pa,
                        const Vec3 &pb, int move, double ta, double tb, CollisionReport &report,
                        std::atomic_bool *cancel = nullptr) const;
    void deposit(const Vec3 &a, const Vec3 &b, double width, double height, Role role,
                 const Vec3 &normal = Vec3::UnitZ(), const QString &objectId = {});
    void set_active_object(const QString &objectId);

  private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
std::vector<JointState> repair_travel(CollisionScene &scene, const MachineProfile &machine,
                                      const ToolheadAssembly &assembly, const JointState &start,
                                      const JointState &goal, std::atomic_bool *cancel = nullptr);
CollisionReport validate_motion(const Project &p, std::atomic_bool *cancel = nullptr,
                                std::function<void(double)> progress = {});
bool repair_motion(Project &p, std::atomic_bool *cancel = nullptr);
} // namespace fiveaxis
