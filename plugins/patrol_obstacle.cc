// Gazebo model plugin: moves a model along a looping trajectory.
//
// All positions are offsets from the model's pose when the plugin loads, so the trajectory
// follows the model wherever it is spawned. Two forms are supported.
//
// Back-and-forth patrol (goes to <offset> in half a period and returns in the other half):
//
//   <plugin name="patrol" filename="libpatrol_obstacle.so">
//     <offset>6 0 0</offset>   <!-- displacement in metres (world frame) -->
//     <period>24</period>      <!-- duration of one round trip in seconds -->
//   </plugin>
//
// Keyframed trajectory (one <keyframe> per waypoint: "time dx dy dz"); the model starts at
// the spawn pose at t = 0 and returns there at t = <period>:
//
//   <plugin name="patrol" filename="libpatrol_obstacle.so">
//     <period>160</period>
//     <keyframe>10 -0.5 -1.0 0</keyframe>
//     <keyframe>50 -3.5 -1.0 0</keyframe>
//   </plugin>

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

class PatrolObstacle : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    double period = sdf->HasElement("period") ? sdf->Get<double>("period") : 20.0;
    if (period <= 0.0) {
      gzerr << "patrol_obstacle: <period> must be positive\n";
      return;
    }

    std::vector<std::pair<double, ignition::math::Vector3d>> keys;
    if (sdf->HasElement("keyframe")) {
      for (sdf::ElementPtr e = sdf->GetElement("keyframe"); e != nullptr;
        e = e->GetNextElement("keyframe"))
      {
        std::istringstream text(e->Get<std::string>());
        double t, x, y, z;
        if (!(text >> t >> x >> y >> z)) {
          gzerr << "patrol_obstacle: a <keyframe> needs 'time dx dy dz'\n";
          return;
        }
        keys.emplace_back(t, ignition::math::Vector3d(x, y, z));
      }
    } else if (sdf->HasElement("offset")) {
      keys.emplace_back(period / 2.0, sdf->Get<ignition::math::Vector3d>("offset"));
    } else {
      gzerr << "patrol_obstacle: give an <offset> or <keyframe> elements\n";
      return;
    }

    std::sort(keys.begin(), keys.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
    if (keys.front().first > 0.0) {
      keys.insert(keys.begin(), {0.0, ignition::math::Vector3d::Zero});
    }
    if (keys.back().first < period) {
      keys.emplace_back(period, ignition::math::Vector3d::Zero);
    }

    const ignition::math::Pose3d start = model->WorldPose();
    common::PoseAnimationPtr animation(
      new common::PoseAnimation("patrol_" + model->GetName(), period, true));
    for (const auto & k : keys) {
      common::PoseKeyFrame * key = animation->CreateKeyFrame(k.first);
      key->Translation(start.Pos() + k.second);
      key->Rotation(start.Rot());
    }
    model->SetAnimation(animation);
  }
};

GZ_REGISTER_MODEL_PLUGIN(PatrolObstacle)

}  // namespace gazebo
