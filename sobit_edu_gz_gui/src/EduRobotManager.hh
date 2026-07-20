#ifndef SOBIT_EDU_GZ_GUI_EDUROBOTMANAGER_HH_
#define SOBIT_EDU_GZ_GUI_EDUROBOTMANAGER_HH_

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTimer>

#include <gz/gui/Plugin.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/rendering/RenderTypes.hh>
#include <gz/transport/Node.hh>

#include "EduLogSink.hh"

namespace edu_gz_gui
{
/// \brief Sidebar plugin that spawns, removes and teleoperates SOBIT EDU
/// from inside the Gazebo GUI.
///
/// Nothing exists until "Spawn" is pressed: spawning starts
/// sobit_edu_bringup/robot.launch.py as a child QProcess (so controllers/
/// bridges come up exactly as they do from a terminal). Removal despawns
/// the gz entity (see setRobotVisible()'s comment for why "despawn" here
/// actually means parking it, not truly removing it) and terminates the
/// owned launch process. Teleop publishes gz.msgs.Twist on /<name>/cmd_vel
/// (bridged to ROS by robot.launch.py itself); a companion standalone
/// keyboard node (scripts/edu_teleop_keyboard.py) can drive the same robot
/// concurrently, mirroring gz_human_sim's human_teleop_switcher.py.
///
/// Ported and trimmed from guider_multifloor_builder's GuiderRobotManager:
/// this package only ever manages one robot type (SOBIT EDU), so the
/// multi-type registry, HSRB and gz_human branches, and the
/// GUIDER_ROBOTS config-spawn path are all gone -- every robot in
/// `robots` was spawned by this panel's own "Spawn" button.
class EduRobotManager : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QStringList robotList READ RobotList NOTIFY robotsChanged)
  Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)

  /// \brief Names of robots that currently have an rviz2 window owned by
  /// this panel (drives the per-row RViz toggle button label).
  Q_PROPERTY(QStringList rvizList READ RvizList NOTIFY RvizChanged)

  /// \brief "name/group" entries for every sensor bridge this panel is
  /// running (group: lidar | color | depth); drives the checkboxes.
  Q_PROPERTY(QStringList sensorList READ SensorList NOTIFY sensorsChanged)

  /// \brief Names of robots currently hidden (parked but kept in the
  /// list; their nodes are stopped). Drives the 表示/非表示 button.
  Q_PROPERTY(QStringList hiddenList READ HiddenList NOTIFY robotsChanged)

  public: EduRobotManager();
  public: ~EduRobotManager() override;

  public: QStringList RobotList() const;
  public: QString Status() const;

  /// \brief Default model name for a new spawn (used by the QML name
  /// field): "sobit_edu", or "sobit_edu_2", "sobit_edu_3", ... if that
  /// name is already taken by a robot this panel spawned.
  public: Q_INVOKABLE QString defaultName() const;

  /// \brief Whether sobit_edu_bringup is installed in this workspace
  /// (checked once at startup against AMENT_PREFIX_PATH). spawnRobot()
  /// refuses to run otherwise, with a clear log line instead of silently
  /// dying inside `ros2 launch`.
  public: Q_INVOKABLE bool available() const;

  /// \brief EDU always spawns with every gz sensor in the model; the
  /// sensors stay idle (gz skips rendering sensors without subscribers)
  /// until setSensor() bridges them, so they can be toggled at runtime.
  public: Q_INVOKABLE void spawnRobot(
      const QString &_name, double _x, double _y, double _z, double _yaw);

  /// \brief Start (_on = true) or stop the sensor bridge node for one
  /// robot (_group: "lidar" | "color" | "depth"). ON boots the bridge
  /// node (gz starts rendering + the ROS topic appears); OFF group-kills
  /// it (topic vanishes). An open rviz2 is left running either way -- its
  /// Display's own Enabled checkbox is not touched -- but with the topic
  /// gone, that Display shows "No Image"/no data regardless of what its
  /// checkbox says, including if the user re-checks it by hand.
  public: Q_INVOKABLE void setSensor(
      int _robotIndex, const QString &_group, bool _on);

  public: QStringList SensorList() const;

  public: Q_INVOKABLE void removeRobot(int _robotIndex);

  /// \brief Hide (_visible = false: stop all its nodes and park the
  /// model out of sight, keeping the list entry as 非表示中) or show
  /// again (_visible = true: relaunch the bringup at the last known
  /// pose).
  public: Q_INVOKABLE void setRobotVisible(int _robotIndex, bool _visible);

  public: QStringList HiddenList() const;

  /// \brief Point the GUI camera at a robot by driving the rendering
  /// camera's follow/track targets directly on the render thread (no
  /// dependency on the CameraTracking plugin's services, whose API
  /// differs between gz-gui versions). _viewIndex matches the QML
  /// viewpoint list: 0 = free camera, 1 = first person, then
  /// chase/front/side/top offsets scaled by _distance meters.
  /// _transitionSeconds controls the initial move (0 = immediate, up to
  /// 2 s), while _followGain controls responsiveness after it settles.
  public: Q_INVOKABLE void setViewpoint(
      int _robotIndex, int _viewIndex, double _distance,
      double _transitionSeconds, double _followGain);

  /// \brief Start (or stop, if running) a per-robot rviz2 window showing
  /// the robot's real sensor streams (head camera image, LiDAR, TF).
  public: Q_INVOKABLE void toggleRviz(int _robotIndex);

  public: QStringList RvizList() const;

  /// \brief One teleop tick (~10 Hz while a QML button is held).
  /// _linear/_lateral in m/s, _angular in rad/s, robot frame.
  public: Q_INVOKABLE void teleopMove(
      int _robotIndex, double _linear, double _lateral, double _angular);

  public: Q_INVOKABLE void teleopStop(int _robotIndex);

  signals: void robotsChanged();
  signals: void StatusChanged();
  signals: void RvizChanged();
  signals: void sensorsChanged();

  /// \brief Marshals messages from gz-transport threads onto the Qt thread.
  signals: void LogRequested(const QString &_level, const QString &_message);

  protected: void LoadConfig(
      const tinyxml2::XMLElement *_pluginElem) override;

  /// \brief Watches gz::gui::events::Render to apply viewpoint commands on
  /// the render thread (the only thread allowed to touch the Ogre2 scene).
  protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

  /// \brief Forwards to the EduOperationLog panel ("ロボット" source).
  /// Runs on the Qt thread (LogRequested is queued there).
  private slots: void AppendLog(
      const QString &_level, const QString &_message);

  private: struct Robot
  {
    std::string name;

    /// The gz model/entity name for this robot. Always equal to `name`:
    /// robot.launch.py spawns the gz entity with -name robot_name, so
    /// this must match `name` exactly for every gz-side operation keyed
    /// by entity identity (the pose cache, FreezeEntity/UnfreezeEntity,
    /// the camera's scene-node lookup in ApplyViewpoint) to actually find
    /// the entity.
    std::string entityName;
    QProcess *process{nullptr};  // owned launch process

    /// False while hidden (nodes stopped, model parked, entry kept).
    bool visible{true};

    /// Spawn pose; refreshed with the last live pose when hiding so a
    /// re-show puts the robot back where it stood.
    double x{0.0};
    double y{0.0};
    double z{0.05};
    double yaw{0.0};
  };

  private: struct CachedPose
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double yaw{0.0};
    bool valid{false};
  };

  /// \brief Pending camera command, written on the Qt thread by
  /// setViewpoint() and consumed on the render thread by
  /// ApplyViewpoint(). `engage` false means "release the camera".
  private: struct ViewCommand
  {
    bool pending{false};
    bool engage{false};
    std::string target;
    gz::math::Vector3d followOffset{0.0, 0.0, 0.0};
    gz::math::Vector3d trackOffset{0.0, 0.0, 0.0};
    double transitionSeconds{0.6};
    double followGain{0.35};
    QString label;
    int retries{0};
  };

  /// \brief Render thread only: apply the pending ViewCommand to the GUI
  /// user camera (retrying while the target model is still loading).
  private: void ApplyViewpoint();

  /// \brief Terminate a process politely, escalating to kill() after a
  /// few seconds. Used for plain (non-setsid) children such as rviz2.
  private: void ShutdownProcess(QProcess *_process);

  /// \brief Signal a setsid-started child's whole process group
  /// (SIGINT -> SIGTERM -> SIGKILL) so every node it spawned dies with it
  /// and no orphan topics survive. Used for launches and bridges.
  private: void TerminateProcessGroup(QProcess *_process);

  /// \brief `ros2 launch` argument list to (re)spawn EDU from its stored
  /// pose; shared by spawnRobot() and setRobotVisible(true). _spawnEntity
  /// is false to reattach ROS nodes to a gz entity that already exists
  /// (see Robot::entityName / FreezeEntity) instead of spawning a new
  /// one -- robot.launch.py's own `spawn_entity` argument exists
  /// specifically for this reattach flow.
  private: QStringList BuildSpawnArguments(
      const Robot &_robot, bool _spawnEntity = true) const;

  /// \brief The rviz2 layout for one robot: sobit_edu_bringup's
  /// rviz/gazebo.rviz with the robot name substituted and the RobotModel
  /// description made transient-local. Every sensor display is left at
  /// the stock layout's own Enabled: true regardless of this panel's
  /// sensor checkbox state (see the .cc). Empty on error.
  private: QString BuildRvizConfig(const Robot &_robot);

  /// \brief Start rviz2 with BuildRvizConfig()'s layout.
  private: void StartRviz(const Robot &_robot);

  /// \brief Ask the window manager (via wmctrl) to keep the rviz2 window
  /// whose title contains _path always-on-top. Retries a few times since
  /// the window does not exist the instant the process starts.
  private: void RaiseRvizAlwaysOnTop(const QString &_path, int _attempt);

  private: void StopRviz(const std::string &_robot);

  private: void DiscoverWorld();
  private: void OnPoseInfo(const gz::msgs::Pose_V &_message);

  /// \brief Publish one gz Twist on /<name>/cmd_vel.
  private: void PublishTwist(
      const Robot &_robot,
      double _linear, double _lateral, double _angular);

  /// \brief Hides EDU's entity without removing it -- see Robot::entityName
  /// for why an actual despawn is unsafe. Tells EduRobotFreezeSystem
  /// (world plugin) to park it far below the world and hold that pose
  /// every physics step, which makes it invisible, non-colliding and
  /// drift-free in one move.
  private: void FreezeEntity(const std::string &_entityName);

  /// \brief Reverses FreezeEntity(): teleports the entity to the given
  /// pose and releases it back to normal physics (and, once ROS nodes
  /// relaunch, its own wheel controllers). The pose rides inside the
  /// unfreeze message -- see the .cc for the release-step race a separate
  /// set_pose call would lose.
  private: void UnfreezeEntity(
      const std::string &_entityName,
      double _x, double _y, double _z, double _yaw);

  /// \brief Deactivate and unload every controller currently loaded in
  /// _robot's controller_manager, blocking until done (a few seconds:
  /// this shells out to `ros2 control` three times in sequence). Called
  /// before FreezeEntity() when hiding/removing: the entity (and its
  /// gz_ros2_control plugin) is parked, not removed (see FreezeEntity),
  /// so its controllers stay loaded and *active* unless explicitly
  /// unloaded here -- otherwise 表示 relaunching robot.launch.py's
  /// controller spawners fails ("can not be configured from 'active'
  /// state") and every controller topic/service stays alive for as long
  /// as the entity is parked, even though the ROS launch side was fully
  /// torn down.
  private: void UnloadRobotControllers(const Robot &_robot);

  private: QProcess *StartLaunchProcess(
      const QString &_summary, const QStringList &_arguments);
  private: void SetStatus(const QString &_status);

  /// \brief First existing file matching _relative under any
  /// AMENT_PREFIX_PATH prefix (empty when none). Shared by the package
  /// availability check and BuildRvizConfig's layout lookup.
  private: static QString ResolveAmentPath(const QString &_relative);

  /// \brief True when share/<package>/package.xml exists under any
  /// AMENT_PREFIX_PATH prefix.
  private: static bool PackageAvailable(const QString &_package);

  private: gz::transport::Node node;
  private: std::map<std::string, gz::transport::Node::Publisher> publishers;

  /// \brief FreezeEntity()/UnfreezeEntity()'s channel to
  /// EduRobotFreezeSystem (world plugin).
  private: gz::transport::Node::Publisher freezePublisher;

  private: std::vector<Robot> robots;
  private: std::string worldName;

  /// \brief Whether sobit_edu_bringup is installed, resolved once in the
  /// constructor.
  private: bool packageAvailable{false};

  private: std::mutex poseMutex;
  private: std::map<std::string, CachedPose> poses;
  private: bool poseSubscribed{false};

  private: QString statusText{"ワールドを検出中…"};
  private: EduLogSink logSink;

  /// \brief Robot the GUI camera currently follows (empty = free camera);
  /// used to cancel the view when that robot is removed (Qt thread).
  private: std::string viewpointTarget;

  /// \brief Guards viewCommand (Qt thread writes, render thread reads).
  private: std::mutex viewMutex;

  private: ViewCommand viewCommand;

  /// \brief GUI user camera, cached on the render thread.
  private: gz::rendering::CameraPtr userCamera;

  // Render-thread state: after the initial camera transition, restore a
  // responsive steady follow gain so smooth switching does not cause lag.
  private: bool followGainPending{false};
  private: std::chrono::steady_clock::time_point followGainDeadline;
  private: double steadyFollowGain{0.35};

  /// \brief rviz2 windows owned by this panel, keyed by robot name
  /// (Qt thread only).
  private: std::map<std::string, QProcess *> rvizProcesses;

  /// \brief Per-sensor ros_gz_bridge processes, keyed by "name/group"
  /// (Qt thread only).
  private: std::map<std::string, QProcess *> sensorBridges;

  /// \brief Stop every sensor bridge belonging to one robot.
  private: void StopRobotSensors(const std::string &_robot);
};
}  // namespace edu_gz_gui

#endif  // SOBIT_EDU_GZ_GUI_EDUROBOTMANAGER_HH_
