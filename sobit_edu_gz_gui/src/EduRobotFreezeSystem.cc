#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace edu_gz_gui
{
// World-level system: while EduRobotManager has the EDU robot 非表示/削除
// ("hidden"), its gz entity is kept alive rather than actually removed --
// despawning one that owns a depth camera and later recreating one
// segfaults gz-sim's Sensors system (an Ogre2 Hlms datablock reuse bug in
// the render thread; ported verbatim from guider_multifloor_builder's
// GuiderRobotFreezeSystem, where this was reproduced reliably even with a
// brand new entity name -- see EduRobotManager.cc's setRobotVisible()).
//
// A hidden entity must be (a) out of sight, (b) unable to collide with
// anything, and (c) not accumulating velocity under gravity. All three
// are solved at once by parking it far below the world and re-asserting
// that pose every physics step:
//  - WorldPoseCmd every PreUpdate pins it at the parked spot exactly (a
//    one-shot set_pose teleport does NOT reset velocity, so gravity's
//    contribution compounds every tick into runaway free-fall);
//  - zeroed Linear/AngularVelocityCmd keeps the base from drifting;
//  - nothing exists at the parked depth, so collision and visibility
//    simply never come up.
//
// "unfreeze:<name>:<x>:<y>:<z>:<yaw>" teleports the entity to that pose
// and removes the command components again, releasing it back to normal
// physics; EduRobotManager then restarts the ROS nodes.
class EduRobotFreezeSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  /// \brief Absolute z the parked entity is held at: far below the
  /// world's floor, so it can neither be seen nor collide with anything.
  public: static constexpr double kParkZ = -100.0;

  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->node.Subscribe(
        "/edu_gz_gui/robot_freeze/config",
        &EduRobotFreezeSystem::OnConfig, this);
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    std::lock_guard<std::mutex> lock(this->mutex);

    // One-shot: teleport back to the requested restore pose and drop the
    // velocity command components so the physics engine (and, once this
    // robot's ROS nodes are relaunched, its own wheel controllers)
    // regains normal control. The restore teleport must be issued HERE,
    // not as a separate set_pose service call from the GUI: UserCommands
    // runs its PreUpdate before this system's, so a set_pose landing on
    // the same step as the unfreeze would have its WorldPoseCmd silently
    // deleted by the cleanup below and the robot would stay parked.
    // Physics consumes a WorldPoseCmd after applying it, so the one
    // written here teleports once and then releases the entity.
    for (const auto &[name, restore] : this->pendingUnfreeze)
    {
      const gz::sim::Entity entity = this->Resolve(_ecm, name);
      if (entity != gz::sim::kNullEntity)
      {
        auto *poseCmd =
          _ecm.Component<gz::sim::components::WorldPoseCmd>(entity);
        if (poseCmd)
          *poseCmd = gz::sim::components::WorldPoseCmd(restore);
        else
          _ecm.CreateComponent(
              entity, gz::sim::components::WorldPoseCmd(restore));
        _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
            entity);
        _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
            entity);
      }
      this->resolved.erase(name);
      this->parkedPoses.erase(name);
    }
    this->pendingUnfreeze.clear();

    static const gz::math::Vector3d kZero = gz::math::Vector3d::Zero;
    for (const std::string &name : this->frozenNames)
    {
      const gz::sim::Entity entity = this->Resolve(_ecm, name);
      if (entity == gz::sim::kNullEntity)
        continue;

      // First frame after "freeze": remember where the robot stood, then
      // park below that spot. Pose capture must happen here (physics
      // thread), not in OnConfig (transport thread).
      auto parked = this->parkedPoses.find(name);
      if (parked == this->parkedPoses.end())
      {
        const gz::math::Pose3d current = gz::sim::worldPose(entity, _ecm);
        parked = this->parkedPoses.emplace(
            name,
            gz::math::Pose3d(
                current.Pos().X(), current.Pos().Y(), kParkZ,
                0.0, 0.0, current.Rot().Yaw())).first;
      }

      // Re-assert pose and zero velocity every step: Physics consumes a
      // WorldPoseCmd after applying it, so a single write only teleports
      // once and gravity takes over again from there.
      auto *poseCmd =
        _ecm.Component<gz::sim::components::WorldPoseCmd>(entity);
      if (poseCmd)
        *poseCmd = gz::sim::components::WorldPoseCmd(parked->second);
      else
        _ecm.CreateComponent(
            entity, gz::sim::components::WorldPoseCmd(parked->second));

      auto *linear =
        _ecm.Component<gz::sim::components::LinearVelocityCmd>(entity);
      if (linear)
        *linear = gz::sim::components::LinearVelocityCmd(kZero);
      else
        _ecm.CreateComponent(
            entity, gz::sim::components::LinearVelocityCmd(kZero));

      auto *angular =
        _ecm.Component<gz::sim::components::AngularVelocityCmd>(entity);
      if (angular)
        *angular = gz::sim::components::AngularVelocityCmd(kZero);
      else
        _ecm.CreateComponent(
            entity, gz::sim::components::AngularVelocityCmd(kZero));
    }
  }

  /// \brief mutex must be held.
  private: gz::sim::Entity Resolve(
      const gz::sim::EntityComponentManager &_ecm, const std::string &_name)
  {
    const auto cached = this->resolved.find(_name);
    if (cached != this->resolved.end())
      return cached->second;

    const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Name(_name), gz::sim::components::Model());
    if (entity != gz::sim::kNullEntity)
      this->resolved[_name] = entity;
    return entity;
  }

  /// \brief "freeze:<name>" parks that entity below the world from the
  /// next PreUpdate on; "unfreeze:<name>:<x>:<y>:<z>:<yaw>" teleports it
  /// to that pose and releases it (entity names never contain ':').
  /// Transport thread; PreUpdate applies these under the same mutex.
  private: void OnConfig(const gz::msgs::StringMsg_V &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    for (int i = 0; i < _msg.data_size(); ++i)
    {
      std::vector<std::string> tokens;
      {
        const std::string &entry = _msg.data(i);
        std::size_t begin = 0;
        while (begin <= entry.size())
        {
          const auto sep = entry.find(':', begin);
          if (sep == std::string::npos)
          {
            tokens.push_back(entry.substr(begin));
            break;
          }
          tokens.push_back(entry.substr(begin, sep - begin));
          begin = sep + 1;
        }
      }
      if (tokens.size() < 2)
        continue;
      const std::string &action = tokens[0];
      const std::string &name = tokens[1];
      if (action == "freeze")
      {
        this->pendingUnfreeze.erase(name);
        this->frozenNames.insert(name);
      }
      else if (action == "unfreeze" && tokens.size() >= 6)
      {
        try
        {
          const double x = std::stod(tokens[2]);
          const double y = std::stod(tokens[3]);
          const double z = std::stod(tokens[4]);
          const double yaw = std::stod(tokens[5]);
          this->frozenNames.erase(name);
          this->pendingUnfreeze[name] =
            gz::math::Pose3d(x, y, z, 0.0, 0.0, yaw);
        }
        catch (const std::exception &)
        {
          continue;  // unparsable number; ignore the entry
        }
      }
    }
  }

  private: gz::transport::Node node;

  /// \brief Guards every container below: OnConfig writes from a
  /// transport thread, PreUpdate reads/applies on the physics thread.
  private: std::mutex mutex;
  private: std::unordered_set<std::string> frozenNames;

  /// \brief Entities to release on the next PreUpdate, mapped to the
  /// world pose each one is teleported back to as it is released.
  private: std::unordered_map<std::string, gz::math::Pose3d>
    pendingUnfreeze;

  /// \brief Where each frozen entity is held (captured on the first
  /// PreUpdate after its freeze request; x/y/yaw from where it stood,
  /// z forced to kParkZ).
  private: std::unordered_map<std::string, gz::math::Pose3d> parkedPoses;

  /// \brief Name -> entity cache (physics thread only).
  private: std::unordered_map<std::string, gz::sim::Entity> resolved;
};
}  // namespace edu_gz_gui

GZ_ADD_PLUGIN(
  edu_gz_gui::EduRobotFreezeSystem,
  gz::sim::System,
  edu_gz_gui::EduRobotFreezeSystem::ISystemConfigure,
  edu_gz_gui::EduRobotFreezeSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  edu_gz_gui::EduRobotFreezeSystem,
  "edu_gz_gui::EduRobotFreezeSystem")
