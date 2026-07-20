#include "EduRobotManager.hh"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iterator>
#include <memory>
#include <thread>

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMetaObject>
#include <QProcessEnvironment>
#include <QPointer>
#include <QTime>
#include <QTimer>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>

namespace edu_gz_gui
{
// This panel only ever manages one robot type. Keeping these as named
// constants (rather than scattering the literals) mirrors
// GuiderRobotManager's per-type traits table, trimmed to the single row
// this package needs; keep in sync with sobit_edu_bringup/launch/
// robot.launch.py and gazebo.urdf.xacro if either changes.
static const char kPackage[] = "sobit_edu_bringup";
static const char kCameraPrefix[] = "head_camera_base";

EduRobotManager::EduRobotManager()
  : gz::gui::Plugin()
{
  this->title = "EDU Robot Manager";

  // Availability is a pure environment question (is sobit_edu_bringup
  // installed in any AMENT_PREFIX_PATH?), so it can be answered before
  // the QML binds to it -- spawnRobot() refuses to run with a clear log
  // line instead of silently dying inside `ros2 launch`.
  this->packageAvailable = PackageAvailable(kPackage);
}

EduRobotManager::~EduRobotManager()
{
  // Launches and bridges run under setsid (their own process groups), so
  // signal the whole group -- terminate() alone would orphan their nodes.
  const auto stopGroup = [](QProcess *_process, int _waitMs)
  {
    if (!_process)
      return;
    const qint64 pid = _process->processId();
    if (pid > 0)
      ::kill(static_cast<pid_t>(-pid), SIGINT);
    _process->waitForFinished(_waitMs);
  };

  for (auto &robot : this->robots)
    stopGroup(robot.process, 3000);

  // Snapshot the processes before waiting on any of them: waitForFinished()
  // pumps the Qt event loop, and each process's `finished` signal handler
  // (see StartRviz()/setSensor()) erases its own entry from this very map
  // the instant that process exits. Iterating the live map while such a
  // handler mutates it mid-loop invalidates the range-for's iterator --
  // this used to segfault in std::_Rb_tree_iterator::operator++() on
  // shutdown whenever a process happened to exit during the wait.
  std::vector<QProcess *> rvizSnapshot;
  rvizSnapshot.reserve(this->rvizProcesses.size());
  for (auto &[name, process] : this->rvizProcesses)
  {
    (void)name;
    rvizSnapshot.push_back(process);
  }
  for (auto *process : rvizSnapshot)
  {
    if (process)
    {
      process->terminate();
      process->waitForFinished(2000);
    }
  }

  std::vector<QProcess *> sensorSnapshot;
  sensorSnapshot.reserve(this->sensorBridges.size());
  for (auto &[key, process] : this->sensorBridges)
  {
    (void)key;
    sensorSnapshot.push_back(process);
  }
  for (auto *process : sensorSnapshot)
    stopGroup(process, 2000);
}

void EduRobotManager::LoadConfig(const tinyxml2::XMLElement *)
{
  QObject::connect(
      this, &EduRobotManager::LogRequested,
      this, &EduRobotManager::AppendLog,
      Qt::QueuedConnection);

  // Viewpoint commands touch the Ogre2 scene, which is only safe from the
  // render thread; watch the Render events like the other panels.
  auto *mainWindow = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  if (mainWindow)
    mainWindow->installEventFilter(this);
  else
    gzerr << "[EduRobotManager] no MainWindow; viewpoint control "
          << "will be unavailable.\n";

  if (!this->packageAvailable)
  {
    this->AppendLog(
        "WARN",
        QString("%1 が見当たりません（ワークスペース未導入のため "
                "spawnできません）")
            .arg(kPackage));
  }

  this->DiscoverWorld();

  this->freezePublisher =
    this->node.Advertise<gz::msgs::StringMsg_V>("/edu_gz_gui/robot_freeze/config");
}

QString EduRobotManager::ResolveAmentPath(const QString &_relative)
{
  const QStringList prefixes =
    QProcessEnvironment::systemEnvironment()
      .value("AMENT_PREFIX_PATH")
      .split(QDir::listSeparator(), Qt::SkipEmptyParts);
  for (const QString &prefix : prefixes)
  {
    const QString candidate = QDir(prefix).filePath(_relative);
    if (QFileInfo(candidate).isFile())
      return candidate;
  }
  return {};
}

bool EduRobotManager::PackageAvailable(const QString &_package)
{
  return !ResolveAmentPath("share/" + _package + "/package.xml").isEmpty();
}

bool EduRobotManager::available() const
{
  return this->packageAvailable;
}

bool EduRobotManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
    this->ApplyViewpoint();

  return QObject::eventFilter(_obj, _event);
}

void EduRobotManager::DiscoverWorld()
{
  // /gazebo/worlds is served by the sim server in the same process group;
  // retry until it responds (the GUI can come up before the server).
  gz::msgs::StringMsg_V worlds;
  bool result{false};
  const bool executed =
      this->node.Request("/gazebo/worlds", 500u, worlds, result);

  if (executed && result && worlds.data_size() > 0)
  {
    this->worldName = worlds.data(0);
    this->SetStatus(
        QString("接続済み · world: %1")
            .arg(QString::fromStdString(this->worldName)));
    this->AppendLog(
        "INFO",
        QString("ワールド %1 を検出しました")
            .arg(QString::fromStdString(this->worldName)));

    if (!this->poseSubscribed)
    {
      const std::string topic = "/world/" + this->worldName + "/pose/info";
      if (this->node.Subscribe(
              topic, &EduRobotManager::OnPoseInfo, this))
      {
        this->poseSubscribed = true;
      }
      else
      {
        this->AppendLog(
            "ERROR",
            QString("%1 の購読に失敗しました")
                .arg(QString::fromStdString(topic)));
      }
    }
    return;
  }

  QTimer::singleShot(1000, this, [this]() { this->DiscoverWorld(); });
}

QStringList EduRobotManager::HiddenList() const
{
  QStringList result;
  for (const auto &robot : this->robots)
  {
    if (!robot.visible)
      result << QString::fromStdString(robot.name);
  }
  return result;
}

void EduRobotManager::setRobotVisible(int _robotIndex, bool _visible)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;

  Robot &robot = this->robots.at(_robotIndex);
  if (robot.visible == _visible)
    return;
  const QString name = QString::fromStdString(robot.name);

  if (!_visible)
  {
    // Remember where the robot last stood so 再表示 puts it back there
    // instead of at its original spawn point.
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      const auto found = this->poses.find(robot.entityName);
      if (found != this->poses.end() && found->second.valid)
      {
        robot.x = found->second.x;
        robot.y = found->second.y;
        robot.z = found->second.z + 0.05;
        robot.yaw = found->second.yaw;
        found->second.valid = false;
      }
    }

    if (robot.entityName == this->viewpointTarget)
      this->setViewpoint(_robotIndex, 0, 1.0, 0.0, 0.35);

    // Park below the world FIRST, before anything else: this is what
    // actually makes the robot fly out of sight and stop falling, and it
    // only needs a gz-transport publish (EduRobotFreezeSystem picks it up
    // on the very next physics step, server-side, independent of this GUI
    // thread). Doing this before the slower steps below means the visible
    // "flew away, stopped falling" effect happens immediately regardless
    // of how long they take. Park, don't remove: actually despawning an
    // entity that owns a depth camera and later recreating one segfaults
    // gz-sim's Sensors system (an Ogre2 Hlms datablock reuse bug). Keeping
    // the entity alive (and its gz_ros2_control plugin / controller_manager)
    // parked out of sight lets 再表示 just restore it and restart ROS
    // nodes -- see BuildSpawnArguments' spawn_entity flag.
    this->FreezeEntity(robot.entityName);

    // Unlike removeRobot(), rviz2 is left running: 非表示 is reversible
    // (再表示 relaunches the same name), so its window just sits at
    // "No Image"/frozen TF until then rather than closing and forcing
    // the user to reopen it.
    this->StopRobotSensors(robot.name);

    if (robot.process)
    {
      // Track robot.name in terminatingNames until the kill escalation
      // has had time to actually finish it off (see
      // TerminateProcessGroup's comment): setRobotVisible(true) below
      // refuses to relaunch under this name until that clears, so a fast
      // 非表示→表示 can't start a second launch overlapping the first
      // one's still-dying nodes.
      this->TerminateProcessGroup(robot.process, robot.name);
      robot.process = nullptr;
    }
    this->publishers.erase(robot.name);

    robot.visible = false;
    this->robotsChanged();
    this->SetStatus(name + " を非表示にしました（コントローラ解放中…）");
    this->AppendLog(
        "OK",
        name + " を非表示にしました（ノード停止・場外へ退避。"
               "「表示」で復活します）");

    // Runs in the background (does not block the Qt thread -- see
    // UnloadRobotControllersAsync) since it shells out to `ros2 control`
    // up to 1 + 1 + N times and each can take several seconds; the robot
    // already flew away and its nodes are already dying above regardless
    // of how long this takes or whether it times out. Needed so a later
    // 再表示 doesn't fail to reconfigure controllers that are still
    // loaded/active from this hide. unloadingNames blocks a hide/show for
    // this name until it's done (see setRobotVisible(true)); the outcome
    // is recorded on the robot (Robot::controllersDirty) by name, since
    // _robotIndex may no longer point at this robot by the time it runs.
    const std::string robotName = robot.name;
    this->unloadingNames.insert(robotName);
    this->UnloadRobotControllersAsync(robotName,
        [this, robotName](bool _ok)
    {
      this->unloadingNames.erase(robotName);
      const auto found = std::find_if(
          this->robots.begin(), this->robots.end(),
          [&robotName](const Robot &_r) { return _r.name == robotName; });
      if (found == this->robots.end())
        return;  // removed while the unload was in flight
      found->controllersDirty = !_ok;
      if (!_ok)
      {
        this->SetStatus(
            QString::fromStdString(robotName) + " のコントローラ解放が未確認です");
      }
    });
    return;
  }

  if (this->terminatingNames.count(robot.name) ||
      this->unloadingNames.count(robot.name))
  {
    this->AppendLog(
        "WARN",
        name + " はまだ前回の処理（ノード終了・コントローラ解放）が"
               "完了していません。数秒待ってから再度「表示」を"
               "押してください");
    this->SetStatus(name + " の後片付け中…少し待ってから再度お試しください");
    return;
  }

  // The rest of showing this robot, factored out so it can run either
  // immediately below or after the controllersDirty retry beneath it
  // completes. Looks the robot back up by name rather than closing over
  // `robot`/`_robotIndex`, since those may be stale by the time an async
  // retry's callback runs.
  const std::string robotName = robot.name;
  auto doShow = [this, robotName]()
  {
    const auto found = std::find_if(
        this->robots.begin(), this->robots.end(),
        [&robotName](const Robot &_r) { return _r.name == robotName; });
    if (found == this->robots.end())
      return;
    Robot &robot = *found;
    const QString name = QString::fromStdString(robot.name);

    // The entity was parked below the world, not removed, while hidden:
    // release it back to the remembered pose instead of spawning a new
    // one (entityName is unchanged). The teleport rides inside the
    // unfreeze message -- see UnfreezeEntity() for why it must not be a
    // separate set_pose call.
    this->UnfreezeEntity(
        robot.entityName, robot.x, robot.y, robot.z, robot.yaw);

    auto *process = this->StartLaunchProcess(
        name, this->BuildSpawnArguments(robot, false));
    if (!process)
      return;
    robot.process = process;
    robot.visible = true;
    this->robotsChanged();
    this->SetStatus(name + " を再表示中…");
    this->AppendLog(
        "OK",
        QString("%1 を再表示しました（ノード再起動、位置 (%2, %3)）")
            .arg(name).arg(robot.x).arg(robot.y));
  };

  if (!robot.controllersDirty)
  {
    doShow();
    return;
  }

  // The hide that parked this robot could not confirm its controllers
  // were deactivated/unloaded; relaunching the spawners now would very
  // likely hit "can not be configured from 'active' state" against a
  // controller_manager that still thinks they're active. Retry the
  // unload here (controller_manager itself has had time to settle since
  // the hide) instead of relaunching straight into a known failure mode.
  this->AppendLog(
      "INFO",
      name + " は前回のコントローラ解放が未確認のため、再表示前に"
             "再試行します");
  this->SetStatus(name + " のコントローラ解放を再試行中…");
  this->unloadingNames.insert(robotName);
  this->UnloadRobotControllersAsync(robotName,
      [this, robotName, doShow](bool _ok)
  {
    this->unloadingNames.erase(robotName);
    const auto found = std::find_if(
        this->robots.begin(), this->robots.end(),
        [&robotName](const Robot &_r) { return _r.name == robotName; });
    if (found == this->robots.end())
      return;
    found->controllersDirty = !_ok;
    if (!_ok)
    {
      const QString qname = QString::fromStdString(robotName);
      this->AppendLog(
          "ERROR",
          qname + " のコントローラを解放できないため再表示を中止しました。"
                  "Gazeboの再起動が必要な場合があります");
      this->SetStatus(qname + " の再表示に失敗しました（コントローラ解放不可）");
      return;
    }
    doShow();
  });
}

QStringList EduRobotManager::RobotList() const
{
  QStringList result;
  for (const auto &robot : this->robots)
    result << QString::fromStdString(robot.name);
  return result;
}

QString EduRobotManager::Status() const
{
  return this->statusText;
}

QString EduRobotManager::defaultName() const
{
  const std::string base = "sobit_edu";

  const auto taken = [this](const std::string &_candidate)
  {
    return std::any_of(
        this->robots.begin(), this->robots.end(),
        [&_candidate](const Robot &_robot)
        { return _robot.name == _candidate; });
  };

  if (!taken(base))
    return QString::fromStdString(base);
  for (int suffix = 2; suffix < 100; ++suffix)
  {
    const std::string candidate = base + "_" + std::to_string(suffix);
    if (!taken(candidate))
      return QString::fromStdString(candidate);
  }
  return QString::fromStdString(base);
}

QProcess *EduRobotManager::StartLaunchProcess(
    const QString &_summary, const QStringList &_arguments)
{
  auto *process = new QProcess(this);
  // setsid makes the child its own session/process-group leader, so
  // TerminateProcessGroup() can signal every descendant node at once.
  // Without it, killing only the `ros2 launch` wrapper left orphaned
  // controller/bridge/robot_state_publisher nodes (and their topics)
  // behind whenever the wrapper died without propagating the shutdown.
  process->setProgram("setsid");
  process->setArguments(QStringList{"ros2"} + _arguments);
  // The bringup output is diagnostic-only here; keep it out of the GUI's
  // stdout but retrievable while debugging.
  process->setStandardOutputFile(QProcess::nullDevice());
  process->setStandardErrorFile(QProcess::nullDevice());
  process->start();
  if (!process->waitForStarted(3000))
  {
    process->deleteLater();
    this->AppendLog("ERROR", _summary + "：ros2 launch を起動できませんでした");
    return nullptr;
  }
  return process;
}

void EduRobotManager::TerminateProcessGroup(
    QProcess *_process, const std::string &_trackName)
{
  if (!_process)
    return;

  if (!_trackName.empty())
    this->terminatingNames.insert(_trackName);

  const qint64 pid = _process->processId();
  if (pid > 0)
  {
    // The child was started under setsid (see StartLaunchProcess), so
    // -pid addresses its whole process group: the launch wrapper AND all
    // the nodes it spawned. SIGINT first (ros2 launch's cleanest path),
    // then escalate for anything that ignores it.
    ::kill(static_cast<pid_t>(-pid), SIGINT);
    QTimer::singleShot(4000, this, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGTERM);
    });
    QTimer::singleShot(8000, this, [pid]()
    {
      if (::kill(static_cast<pid_t>(-pid), 0) == 0)
        ::kill(static_cast<pid_t>(-pid), SIGKILL);
    });
  }
  // Deleting a running QProcess kills only the direct child; delay it
  // until the group signals above have done the real work. Release the
  // name guard (if any) at the same point: by now SIGKILL has had ~1s to
  // land even in the worst case, so the group is reliably gone.
  QTimer::singleShot(9000, this,
      [this, guard = QPointer<QProcess>(_process), _trackName]()
  {
    if (guard)
      guard->deleteLater();
    if (!_trackName.empty())
      this->terminatingNames.erase(_trackName);
  });
}

void EduRobotManager::spawnRobot(
    const QString &_name, double _x, double _y, double _z, double _yaw)
{
  if (!this->packageAvailable)
  {
    this->AppendLog(
        "ERROR",
        QString("EDUのロボットが見当たりません：%1 がワークスペースに"
                "ありません（src 直下に導入してビルドしてください）")
            .arg(kPackage));
    this->SetStatus(
        QString("ロボットが見当たりません（%1 未導入）").arg(kPackage));
    return;
  }
  if (this->worldName.empty())
  {
    this->AppendLog("ERROR", "ワールド未検出のためspawnできません");
    return;
  }

  const QString name = _name.trimmed();
  if (name.isEmpty())
  {
    this->AppendLog("ERROR", "ロボット名を入力してください");
    return;
  }
  const std::string nameStd = name.toStdString();
  const bool duplicated = std::any_of(
      this->robots.begin(), this->robots.end(),
      [&nameStd](const Robot &_robot) { return _robot.name == nameStd; });
  if (duplicated)
  {
    this->AppendLog(
        "ERROR", QString("%1 は既に存在します。別名にしてください").arg(name));
    return;
  }
  if (this->terminatingNames.count(nameStd) ||
      this->unloadingNames.count(nameStd))
  {
    // A robot removed under this same name is still shutting down (see
    // TerminateProcessGroup) or still has an UnloadRobotControllersAsync
    // chain running against its controller_manager namespace: spawning
    // now would start a second robot.launch.py in the same namespace
    // while the old one's nodes/controller cleanup are still in flight.
    this->AppendLog(
        "ERROR",
        name + " は削除処理が完了していません。数秒待ってから"
               "再度お試しください");
    return;
  }

  Robot robot;
  robot.name = nameStd;
  // entityName is always robot.name: robot.launch.py spawns the gz
  // entity with -name robot_name, never a separately-tracked name, so
  // entityName must match that exactly for every gz-side lookup keyed by
  // it (the pose cache, FreezeEntity/UnfreezeEntity, and the camera's
  // scene-node lookup in ApplyViewpoint) to actually find the entity.
  robot.entityName = nameStd;
  robot.x = _x;
  robot.y = _y;
  robot.z = _z;
  robot.yaw = _yaw;

  auto *process =
    this->StartLaunchProcess(name, this->BuildSpawnArguments(robot));
  if (!process)
    return;

  robot.process = process;
  this->robots.push_back(robot);
  {
    // Track this robot's live pose from now on (the hide/show toggle
    // needs the last known position).
    std::lock_guard<std::mutex> lock(this->poseMutex);
    this->poses.try_emplace(robot.entityName);
  }
  this->robotsChanged();
  this->SetStatus(name + " をspawn中…");
  this->AppendLog(
      "OK",
      QString("%1 を (%2, %3, %4) yaw=%5 にspawnしました")
          .arg(name).arg(_x).arg(_y).arg(_z).arg(_yaw));

  // Land the camera on the newly spawned robot right away: view index 7
  // is "右奥上から（全体俯瞰）" (see the enum in setViewpoint(), which must
  // stay in sync with the QML viewpoint ComboBox order). The gz entity
  // may still be a few seconds from existing; ApplyViewpoint() already
  // retries until the render scene has it.
  this->setViewpoint(
      static_cast<int>(this->robots.size()) - 1, 7, 2.0, 0.6, 0.35);
}

QStringList EduRobotManager::BuildSpawnArguments(
    const Robot &_robot, bool _spawnEntity) const
{
  const QString name = QString::fromStdString(_robot.name);
  const QString x = QString::number(_robot.x);
  const QString y = QString::number(_robot.y);
  const QString z = QString::number(_robot.z);
  const QString yaw = QString::number(_robot.yaw);

  // NOTE: these three flags are NOT just "bridge on/off" switches -- in
  // sobit_edu_description's gazebo.urdf.xacro each one wraps its own
  // <xacro:if>, so passing False omits that <sensor> tag from the spawned
  // SDF entirely (camera color/depth simply do not exist; gz's lidar
  // falls back to the unconditional real-hardware lidar def, which
  // publishes on an unnamespaced "scan" topic that never matches this
  // panel's "<name>/scan" bridge). They must stay True so the sensors
  // exist at all -- setSensor() then starts/stops the bridge node, which
  // is the actual runtime on/off switch (see its comment). Traded off:
  // those sensors are <always_on>1</always_on> in the SDF, so they render
  // continuously regardless of whether any bridge is running, not lazily
  // on first subscriber as originally intended -- a real GPU cost this
  // panel cannot control from here.
  QStringList arguments;
  arguments << "launch" << kPackage << "robot.launch.py"
            << "robot_name:=" + name
            << "spawn_entity:=" + QString(_spawnEntity ? "True" : "False")
            << "enable_gz:=True"
            << "robot_coords_x:=" + x
            << "robot_coords_y:=" + y
            << "robot_coords_z:=" + z
            << "robot_coords_Y:=" + yaw
            << "enable_gz_lidar:=True"
            << "enable_gz_head_cam_color:=True"
            << "enable_gz_head_cam_depth:=True";
  return arguments;
}

void EduRobotManager::setViewpoint(
    int _robotIndex, int _viewIndex, double _distance,
    double _transitionSeconds, double _followGain)
{
  // Keep the indices in sync with the QML viewpoint ComboBox.
  enum ViewIndex
  {
    kViewFree = 0,
    kViewFirstPerson,
    kViewBehind,
    kViewFront,
    kViewRight,
    kViewLeft,
    kViewTop,
    kViewFrontRightUp,
    kViewFrontLeftUp,
    kViewCount,
  };
  static const char *kViewLabels[kViewCount] = {
    "自由視点", "一人称（ロボット視点）", "後方追従", "前方から",
    "右横から", "左横から", "俯瞰（真上）",
    "右奥上から（全体俯瞰）", "左奥上から（全体俯瞰）",
  };

  if (_viewIndex < 0 || _viewIndex >= kViewCount)
    return;

  if (_viewIndex == kViewFree)
  {
    if (this->viewpointTarget.empty())
      return;
    this->viewpointTarget.clear();

    std::lock_guard<std::mutex> lock(this->viewMutex);
    this->viewCommand = ViewCommand();
    this->viewCommand.pending = true;
    this->viewCommand.engage = false;
    this->viewCommand.label = "カメラを自由視点に戻しました";
    return;
  }

  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
  {
    this->AppendLog("ERROR", "視点変更：対象ロボットがありません");
    return;
  }

  const double distance = std::clamp(_distance, 0.3, 50.0);
  // Eye level scales with distance: near views sit low, far views look
  // down a little.
  const double height = std::clamp(distance * 0.5, 0.4, 2.5);

  ViewCommand command;
  command.pending = true;
  command.engage = true;
  // Scene nodes are named after the gz entity, not the stable ROS name
  // (see Robot::entityName).
  command.target = this->robots.at(_robotIndex).entityName;
  // Chase views keep looking slightly above the robot's base.
  command.trackOffset = {0.0, 0.0, 0.6};
  command.transitionSeconds = std::clamp(_transitionSeconds, 0.0, 2.0);
  command.followGain = std::clamp(_followGain, 0.05, 1.0);

  switch (_viewIndex)
  {
    case kViewFirstPerson:
      // Camera just above the robot's head, aimed at a point ahead of it
      // in its own frame: turns with the robot like its own eyes. The
      // distance box is not used here.
      command.followOffset = {0.15, 0.0, 1.05};
      command.trackOffset = {3.0, 0.0, 0.95};
      break;
    case kViewBehind:
      command.followOffset = {-distance, 0.0, height};
      break;
    case kViewFront:
      command.followOffset = {distance, 0.0, height};
      break;
    case kViewRight:
      command.followOffset = {0.0, -distance, height};
      break;
    case kViewLeft:
      command.followOffset = {0.0, distance, height};
      break;
    case kViewTop:
      // A touch of forward offset avoids the straight-down singularity.
      command.followOffset = {std::max(0.3, distance * 0.1), 0.0, distance};
      command.trackOffset = {0.0, 0.0, 0.0};
      break;
    case kViewFrontRightUp:
    case kViewFrontLeftUp:
    {
      // Diagonal hero/overview angle: in front of the robot, off to one
      // side, and elevated well above eye level so the whole body (not
      // just the face) fits in frame. distance sets the overall zoom;
      // the front/side/up components share it evenly.
      const double lateral = _viewIndex == kViewFrontRightUp
        ? -distance * 0.7 : distance * 0.7;
      const double up = std::clamp(distance * 0.8, 0.6, 4.0);
      command.followOffset = {distance * 0.7, lateral, up};
      break;
    }
    default:
      return;
  }

  command.label = QString("カメラ視点：%1（%2, %3m）")
    .arg(kViewLabels[_viewIndex],
         QString::fromStdString(this->robots.at(_robotIndex).name))
    .arg(distance, 0, 'f', 1);

  this->viewpointTarget = command.target;

  std::lock_guard<std::mutex> lock(this->viewMutex);
  this->viewCommand = command;
}

void EduRobotManager::ApplyViewpoint()
{
  ViewCommand command;
  bool hasCommand = false;
  {
    std::lock_guard<std::mutex> lock(this->viewMutex);
    hasCommand = this->viewCommand.pending;
    if (hasCommand)
      command = this->viewCommand;
  }
  if (!hasCommand && !this->followGainPending)
    return;

  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  if (!scene)
    return;

  // The MinimalScene plugin tags the GUI camera with this user data.
  if (!this->userCamera)
  {
    for (unsigned int i = 0; i < scene->NodeCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<gz::rendering::Camera>(
          scene->NodeByIndex(i));
      if (!camera || !camera->HasUserData("user-camera"))
        continue;
      const auto data = camera->UserData("user-camera");
      const auto *flag = std::get_if<bool>(&data);
      if (flag && *flag)
      {
        this->userCamera = camera;
        break;
      }
    }
    if (!this->userCamera)
      return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (this->followGainPending && now >= this->followGainDeadline)
  {
    this->userCamera->SetFollowPGain(this->steadyFollowGain);
    this->userCamera->SetTrackPGain(this->steadyFollowGain);
    this->followGainPending = false;
  }
  if (!hasCommand)
    return;

  const auto finish = [this]()
  {
    std::lock_guard<std::mutex> lock(this->viewMutex);
    this->viewCommand.pending = false;
  };

  if (!command.engage)
  {
    this->userCamera->SetFollowTarget(nullptr);
    this->userCamera->SetTrackTarget(nullptr);
    this->followGainPending = false;
    finish();
    this->LogRequested("OK", command.label);
    return;
  }

  // Rendering node names may carry "id::" scope prefixes; accept both the
  // plain model name and a scoped suffix match.
  gz::rendering::NodePtr target = scene->NodeByName(command.target);
  if (!target)
  {
    const std::string suffix = "::" + command.target;
    for (unsigned int i = 0; i < scene->VisualCount(); ++i)
    {
      auto visual = scene->VisualByIndex(i);
      if (!visual)
        continue;
      const std::string &name = visual->Name();
      if (name.size() >= suffix.size() &&
          name.compare(name.size() - suffix.size(), suffix.size(),
              suffix) == 0)
      {
        target = visual;
        break;
      }
    }
  }

  if (!target)
  {
    // The model may still be spawning; retry for ~10 s of frames.
    std::lock_guard<std::mutex> lock(this->viewMutex);
    if (++this->viewCommand.retries > 600)
    {
      this->viewCommand.pending = false;
      this->LogRequested(
          "ERROR",
          QString("視点変更：%1 が見つかりません")
            .arg(QString::fromStdString(command.target)));
    }
    return;
  }

  // Offsets are in the robot's local frame (worldFrame = false), so every
  // view turns together with the robot.
  const double transitionGain = command.transitionSeconds <= 0.0
    ? 1.0
    : std::clamp(
        1.0 - std::pow(
            0.05, 1.0 / std::max(1.0, 60.0 * command.transitionSeconds)),
        0.01, 1.0);

  this->userCamera->SetFollowTarget(target, command.followOffset, false);
  this->userCamera->SetFollowPGain(transitionGain);
  this->userCamera->SetTrackTarget(target, command.trackOffset, false);
  this->userCamera->SetTrackPGain(transitionGain);

  this->steadyFollowGain = command.followGain;
  this->followGainPending = true;
  const auto settleDelay = command.transitionSeconds <= 0.0
    ? std::chrono::milliseconds(50)
    : std::chrono::milliseconds(
        static_cast<int>(command.transitionSeconds * 1000.0));
  this->followGainDeadline = now + settleDelay;
  finish();
  this->LogRequested("OK", command.label);
}

QStringList EduRobotManager::SensorList() const
{
  QStringList result;
  for (const auto &[key, process] : this->sensorBridges)
  {
    (void)process;
    result << QString::fromStdString(key);
  }
  return result;
}

void EduRobotManager::setSensor(
    int _robotIndex, const QString &_group, bool _on)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;

  const Robot &robot = this->robots.at(_robotIndex);
  if (!robot.visible)
  {
    this->AppendLog(
        "ERROR", "非表示中のロボットのセンサーはONにできません");
    this->sensorsChanged();
    return;
  }
  const std::string key = robot.name + "/" + _group.toStdString();
  const bool running =
    this->sensorBridges.find(key) != this->sensorBridges.end();
  if (_on == running)
    return;

  if (!_on)
  {
    // Erase before terminating so an immediate re-ON never collides with
    // the dying process. The bridge node is group-killed, so the topic
    // really disappears from the ROS graph.
    QProcess *process = this->sensorBridges.at(key);
    this->sensorBridges.erase(key);
    this->sensorsChanged();
    this->TerminateProcessGroup(process);
    this->AppendLog(
        "INFO", QString("%1 のセンサー(%2)をOFFにしました（ノード停止）")
          .arg(QString::fromStdString(robot.name), _group));

    // rviz2 is left running (not restarted): with the bridge node gone,
    // the topic really stops, so the matching Display shows "No Image" /
    // "no data" on its own -- even if the user manually re-checks that
    // Display's Enabled box in rviz2, since there is nothing publishing
    // for it to subscribe to.
    return;
  }

  const QString name = QString::fromStdString(robot.name);
  const QString camera = kCameraPrefix;
  QString label;

  // Topic groups mirror robot.launch.py's bridge list; starting the
  // bridge subscribes the gz sensor, which makes gz begin
  // rendering/publishing it.
  QStringList topics;
  QStringList remaps;
  if (_group == "lidar")
  {
    label = "LiDAR";
    // gazebo.urdf.xacro declares <topic>{name}/scan</topic> on the lidar
    // sensor, but gz-sim actually publishes GPU lidar scans under
    // "{name}/lidar/scan" regardless -- <topic> is not honored for this
    // sensor type. Bridge the real gz topic, then remap the ROS side
    // back to ".../scan" so it still matches the real-hardware urg_node
    // topic name that rviz configs key off.
    topics << "/" + name +
      "/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan";
    remaps << "/" + name + "/lidar/scan" + ":=/" + name + "/scan";
  }
  else if (_group == "color")
  {
    label = "RGBカメラ";
    topics << "/" + name + "/" + camera +
        "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
      << "/" + name + "/" + camera +
        "/color@sensor_msgs/msg/Image[gz.msgs.Image";
  }
  else if (_group == "depth")
  {
    label = "深度カメラ";
    topics << "/" + name + "/" + camera +
        "/depth@sensor_msgs/msg/Image[gz.msgs.Image"
      << "/" + name + "/" + camera +
        "/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked";
  }
  else
  {
    return;
  }

  auto *process = new QProcess(this);
  // setsid for the same reason as StartLaunchProcess: `ros2 run` wraps
  // the real parameter_bridge binary, and group signals are the only way
  // to guarantee the bridge itself dies (and its topics vanish).
  process->setProgram("setsid");
  QString nodeName = name + "_" + _group + "_bridge";
  nodeName.replace('-', '_');
  QStringList arguments;
  arguments << "ros2" << "run" << "ros_gz_bridge" << "parameter_bridge";
  arguments += topics;
  arguments << "--ros-args" << "-r" << "__node:=" + nodeName;
  for (const QString &remap : remaps)
    arguments << "-r" << remap;
  process->setArguments(arguments);
  process->setProcessChannelMode(QProcess::MergedChannels);

  // Intentional stops erase the map entry first (see the _on == false
  // branch); this handler then only cleans up after unexpected exits.
  QObject::connect(
      process,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      this,
      [this, key, label, name, process](int, QProcess::ExitStatus)
      {
        const auto found = this->sensorBridges.find(key);
        if (found != this->sensorBridges.end() && found->second == process)
        {
          this->sensorBridges.erase(found);
          this->sensorsChanged();
          this->AppendLog(
              "WARN",
              QString("%1 の%2ブリッジが終了しました").arg(name, label));
        }
        process->deleteLater();
      });

  process->start();
  if (!process->waitForStarted(3000))
  {
    this->AppendLog(
        "ERROR", QString("%1 の%2ブリッジを起動できませんでした")
          .arg(name, label));
    process->deleteLater();
    this->sensorsChanged();
    return;
  }

  this->sensorBridges[key] = process;
  this->sensorsChanged();
  this->AppendLog(
      "OK",
      QString("%1 の%2をONにしました（ノード起動・配信開始）")
        .arg(name, label));

  // rviz2 is left running as-is: the Display's own Enabled checkbox is
  // whatever it was left at, but the bridge is back so its topic is live
  // again. If the user (or ApplySensorDisplayStates at rviz2's next
  // manual open) has it checked, data now flows into it.
}

void EduRobotManager::StopRobotSensors(const std::string &_robot)
{
  const std::string prefix = _robot + "/";
  for (auto iterator = this->sensorBridges.begin();
       iterator != this->sensorBridges.end();)
  {
    if (iterator->first.rfind(prefix, 0) == 0)
    {
      this->TerminateProcessGroup(iterator->second);
      iterator = this->sensorBridges.erase(iterator);
    }
    else
    {
      ++iterator;
    }
  }
  this->sensorsChanged();
}

QStringList EduRobotManager::RvizList() const
{
  QStringList result;
  for (const auto &[name, process] : this->rvizProcesses)
  {
    (void)process;
    result << QString::fromStdString(name);
  }
  return result;
}

void EduRobotManager::toggleRviz(int _robotIndex)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;

  const Robot &robot = this->robots.at(_robotIndex);
  if (this->rvizProcesses.find(robot.name) != this->rvizProcesses.end())
  {
    this->StopRviz(robot.name);
    return;
  }

  if (!robot.visible)
  {
    this->AppendLog(
        "ERROR", "非表示中のロボットのrviz2は起動できません");
    return;
  }

  this->StartRviz(robot);
}

void EduRobotManager::ShutdownProcess(QProcess *_process)
{
  if (!_process)
    return;
  _process->terminate();
  QTimer::singleShot(3000, this, [guard = QPointer<QProcess>(_process)]()
  {
    if (guard && guard->state() != QProcess::NotRunning)
      guard->kill();
  });
}

QString EduRobotManager::BuildRvizConfig(const Robot &_robot)
{
  const QString robot = QString::fromStdString(_robot.name);
  const QString relativePath =
    QDir("share").filePath(QString(kPackage) + "/rviz/gazebo.rviz");

  // Reuse the normal Gazebo RViz2 layout from the robot package.
  const QString sourcePath = ResolveAmentPath(relativePath);

  if (sourcePath.isEmpty())
  {
    this->AppendLog(
        "ERROR",
        QString("標準RViz2設定が見つかりません: %1").arg(relativePath));
    return {};
  }

  QFile source(sourcePath);
  if (!source.open(QIODevice::ReadOnly))
  {
    this->AppendLog(
        "ERROR", QString("標準RViz2設定を読めません: %1").arg(sourcePath));
    return {};
  }

  QString config = QString::fromUtf8(source.readAll());
  source.close();

  // Replace the default robot name in topics, frames and TF Prefix.
  config.replace("sobit_edu", robot);

  // RViz2 starts after robot_state_publisher in this workflow. Make only
  // the RobotModel description subscription transient-local so it receives
  // the already-published URDF; all standard panels and views stay intact.
  const QString descriptionMarker = "Description Topic:";
  const QString durabilityVolatile = "Durability Policy: Volatile";
  const QString robotDescription = "/" + robot + "/robot_description";
  const int descriptionPos = config.indexOf(descriptionMarker);
  const int topicPos = config.indexOf(robotDescription, descriptionPos);
  const int durabilityPos = config.indexOf(
      durabilityVolatile, descriptionPos);
  if (descriptionPos >= 0 && durabilityPos >= descriptionPos &&
      topicPos > durabilityPos)
  {
    config.replace(
        durabilityPos, durabilityVolatile.size(),
        "Durability Policy: Transient Local");
  }

  // Sensor displays (Image, LaserScan, ...) are left exactly as the stock
  // layout ships them regardless of whether this panel's sensor
  // checkboxes currently have that bridge on. An OFF sensor simply has no
  // topic to show, so its Display sits at rviz2's own "No Image"/no-data
  // state; this is preferred over reflecting the bridge state into the
  // checkbox, since rviz2 has no runtime API to update a checkbox after
  // the fact, and toggling the sensor later while this rviz2 stays open
  // (see setSensor()) would otherwise leave stale checkbox state with no
  // way to reconcile it.
  return config;
}

void EduRobotManager::StartRviz(const Robot &_robot)
{
  const QString robot = QString::fromStdString(_robot.name);
  const QString config = this->BuildRvizConfig(_robot);
  if (config.isEmpty())
    return;

  const QString path =
    QDir::temp().filePath("edu_rviz_" + robot + ".rviz");
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
  {
    this->AppendLog(
        "ERROR", QString("rviz2設定を書き出せません: %1").arg(path));
    return;
  }
  file.write(config.toUtf8());
  file.close();

  auto *process = new QProcess(this);
  process->setProgram("rviz2");
  // The robot nodes run with sim time, so rviz2 must too (TF timestamps).
  QString nodeName = robot;
  nodeName.replace("-", "_");
  process->setArguments(
      {"-d", path,
       // Ogre sizes its render targets (G-buffer, depth pre-pass, picking
       // buffer, ...) off the window's pixel size, not a fixed budget: left
       // to auto-maximize on a 4K/high-refresh display, rviz2's window
       // alone can use several GiB of VRAM next to gz-sim's own GUI
       // viewport, and the two together can crash on a smaller card. A
       // modest fixed size keeps rviz2 usable (still resizable afterwards)
       // without that spike. Qt's own flag -- must come before
       // --ros-args, which hands everything after it to rclcpp instead.
       "--qwindowgeometry", "1280x800+80+80",
       "--ros-args", "-p", "use_sim_time:=true",
       "-r", "__node:=" + nodeName + "_rviz2"});
  process->setProcessChannelMode(QProcess::MergedChannels);

  // Nothing previously read this (MergedChannels alone doesn't log
  // anywhere) so an instant crash left no trace beyond "it closed" -- the
  // actual Ogre/Qt/GL error that precedes a crash is normally printed to
  // stdout/stderr right before it dies. Keep only a tail: the crash reason
  // is in the last few lines, not the startup chatter.
  auto outputTail = std::make_shared<QByteArray>();
  QObject::connect(
      process, &QProcess::readyReadStandardOutput, this,
      [process, outputTail]()
      {
        outputTail->append(process->readAllStandardOutput());
        constexpr int kMaxTail = 4000;
        if (outputTail->size() > kMaxTail)
          *outputTail = outputTail->right(kMaxTail);
      });

  // Intentional stops erase the map entry before terminating (so a
  // restart can register a fresh process immediately); this handler then
  // only cleans up after rviz2 windows closed from outside.
  const std::string key = _robot.name;
  QObject::connect(
      process,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      this,
      [this, key, process, outputTail](
          int _exitCode, QProcess::ExitStatus _exitStatus)
      {
        const auto found = this->rvizProcesses.find(key);
        if (found != this->rvizProcesses.end() && found->second == process)
        {
          this->rvizProcesses.erase(found);
          this->RvizChanged();
          if (_exitStatus == QProcess::CrashExit || _exitCode != 0)
          {
            this->AppendLog(
                "ERROR",
                QString("%1 のrviz2が異常終了しました（%2）:\n%3")
                  .arg(
                      QString::fromStdString(key),
                      _exitStatus == QProcess::CrashExit
                        ? QString("crash")
                        : QString("exit code %1").arg(_exitCode),
                      QString::fromUtf8(*outputTail).trimmed()));
          }
          else
          {
            this->AppendLog(
                "INFO",
                QString("%1 のrviz2を終了しました")
                  .arg(QString::fromStdString(key)));
          }
        }
        process->deleteLater();
      });

  process->start();
  if (!process->waitForStarted(3000))
  {
    this->AppendLog(
        "ERROR",
        "rviz2 を起動できませんでした（rviz2 がインストールされているか"
        "確認してください）");
    process->deleteLater();
    return;
  }

  this->rvizProcesses[key] = process;
  this->RvizChanged();
  this->AppendLog(
      "OK", robot + " の標準レイアウトのrviz2を起動しました");

  // rviz2's window title includes this config path, so it is a unique,
  // per-robot wmctrl match string. The window does not exist the instant
  // the process starts, so retry a few times.
  this->RaiseRvizAlwaysOnTop(path, 0);
}

void EduRobotManager::RaiseRvizAlwaysOnTop(const QString &_path, int _attempt)
{
  auto *wmctrl = new QProcess(this);
  QObject::connect(
      wmctrl,
      QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
      this,
      [this, wmctrl, _path, _attempt](int _exitCode, QProcess::ExitStatus)
      {
        if (_exitCode != 0 && _attempt < 8)
        {
          QTimer::singleShot(500, this, [this, _path, _attempt]()
          {
            this->RaiseRvizAlwaysOnTop(_path, _attempt + 1);
          });
        }
        wmctrl->deleteLater();
      });
  wmctrl->start("wmctrl", {"-r", _path, "-b", "add,above"});
}

void EduRobotManager::StopRviz(const std::string &_robot)
{
  const auto found = this->rvizProcesses.find(_robot);
  if (found == this->rvizProcesses.end())
    return;

  QProcess *process = found->second;
  this->rvizProcesses.erase(found);
  this->RvizChanged();
  this->ShutdownProcess(process);
}

void EduRobotManager::removeRobot(int _robotIndex)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;
  if (this->worldName.empty())
  {
    this->AppendLog("ERROR", "ワールド未検出のため削除できません");
    return;
  }

  Robot robot = this->robots.at(_robotIndex);
  const QString name = QString::fromStdString(robot.name);

  // Followed robot about to disappear: release the GUI camera first, and
  // close its rviz2 window / sensor bridges if this panel opened any.
  if (robot.entityName == this->viewpointTarget)
    this->setViewpoint(_robotIndex, 0, 1.0, 0.0, 0.35);
  this->StopRviz(robot.name);
  this->StopRobotSensors(robot.name);

  if (robot.process)
  {
    // Group kill (see TerminateProcessGroup): takes the launch wrapper
    // AND every node it spawned down together, so no orphan topics
    // survive the removal. Track robot.name so spawnRobot() refuses to
    // reuse it until the old process group is confirmed gone -- the
    // robots list entry is erased below, so without this guard a
    // same-named respawn right after removal would race the still-dying
    // old nodes just like the setRobotVisible() hide/show case.
    this->TerminateProcessGroup(robot.process, robot.name);
  }

  // Park below the world, don't remove: see setRobotVisible()'s comment
  // on why despawning and later recreating one segfaults gz-sim when a
  // depth camera is rebuilt. A robot already hidden is parked already;
  // only a still-visible one needs parking now. The entity stays parked
  // forever after this -- there is no way to bring it back once its list
  // entry is gone. FreezeEntity before the (async, non-blocking --
  // UnloadRobotControllersAsync) unload below, same reasoning as
  // setRobotVisible(): the fly-away is one fast publish, so it happens
  // before the potentially multi-second `ros2 control` calls.
  if (robot.visible)
  {
    this->FreezeEntity(robot.entityName);
    // Fire-and-forget: `robot` was copied by value above and the list
    // entry is erased below, so there is no Robot state left to update
    // by the time this finishes (unlike the hide/show case) -- just let
    // it log its own outcome.
    this->UnloadRobotControllersAsync(robot.name, [](bool) {});
  }

  this->publishers.erase(robot.name);
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    this->poses.erase(robot.entityName);
  }
  this->robots.erase(this->robots.begin() + _robotIndex);
  this->robotsChanged();
  this->SetStatus(name + " を削除しました");
}

void EduRobotManager::OnPoseInfo(const gz::msgs::Pose_V &_message)
{
  // Transport thread: only touch the mutex-guarded cache.
  std::lock_guard<std::mutex> lock(this->poseMutex);
  for (const auto &pose : _message.pose())
  {
    auto found = this->poses.find(pose.name());
    if (found == this->poses.end())
      continue;

    auto &cached = found->second;
    cached.x = pose.position().x();
    cached.y = pose.position().y();
    cached.z = pose.position().z();
    const auto &q = pose.orientation();
    cached.yaw = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    cached.valid = true;
  }
}

void EduRobotManager::PublishTwist(
    const Robot &_robot,
    double _linear, double _lateral, double _angular)
{
  auto found = this->publishers.find(_robot.name);
  if (found == this->publishers.end())
  {
    found = this->publishers.emplace(
        _robot.name,
        this->node.Advertise<gz::msgs::Twist>(
            "/" + _robot.name + "/cmd_vel"))
        .first;
  }

  gz::msgs::Twist message;
  message.mutable_linear()->set_x(_linear);
  message.mutable_linear()->set_y(_lateral);
  // EDU's diff-drive controller is a REP-103-compliant passthrough
  // (verified against the running sim), so no sign flip is needed.
  message.mutable_angular()->set_z(_angular);
  found->second.Publish(message);
}

void EduRobotManager::FreezeEntity(const std::string &_entityName)
{
  // EduRobotFreezeSystem does all the work server-side: it parks the
  // entity far below the world and re-asserts that pose (plus zero
  // velocity) every physics step, which covers invisibility, collisions
  // and drift in one move. (The stock per-entity services were dead ends
  // for the "hide in place" variants of this: visual_config/
  // disable_collision reject MODEL-type requests outright, and
  // per-visual visual_config requests, while accepted, had no visible
  // effect.)
  gz::msgs::StringMsg_V freezeMsg;
  freezeMsg.add_data("freeze:" + _entityName);
  this->freezePublisher.Publish(freezeMsg);
}

void EduRobotManager::UnfreezeEntity(
    const std::string &_entityName,
    double _x, double _y, double _z, double _yaw)
{
  // The restore pose rides along in the message: EduRobotFreezeSystem
  // issues the teleport itself on the release step. A separate set_pose
  // service call would race it -- UserCommands' PreUpdate runs before the
  // freeze system's, so a set_pose landing on the release step would be
  // wiped out and the robot would stay parked below the world.
  gz::msgs::StringMsg_V unfreezeMsg;
  unfreezeMsg.add_data(
      "unfreeze:" + _entityName +
      ":" + std::to_string(_x) + ":" + std::to_string(_y) +
      ":" + std::to_string(_z) + ":" + std::to_string(_yaw));
  this->freezePublisher.Publish(unfreezeMsg);
}

bool EduRobotManager::UnloadRobotControllersBlocking(
    const std::string &_robotName,
    std::vector<std::pair<QString, QString>> &_log)
{
  // Runs on a worker thread (see UnloadRobotControllersAsync) -- must not
  // touch `this` or call AppendLog() directly (Qt widget/QML state is
  // main-thread-only). Log lines are appended to _log and replayed on the
  // Qt thread once this returns.
  const QString controllerManager =
      QString::fromStdString("/" + _robotName + "/controller_manager");
  const QString qname = QString::fromStdString(_robotName);

  // Discover what is actually loaded rather than assuming a fixed set.
  // -s/--use-sim-time: this CLI's own short-lived node otherwise runs on
  // wall-clock time while controller_manager (spawned with
  // use_sim_time:=true) runs on sim time -- under a real_time_factor < 1
  // that skew makes wall-clock timeouts fire well before the sim-time
  // equivalent has actually elapsed. QProcess's blocking start()+
  // waitForFinished() API is safe to use off the Qt thread (no event
  // loop required on this thread for it).
  QProcess list;
  list.start("ros2", {"control", "list_controllers",
      "-c", controllerManager, "--spin-time", "2", "-s"});
  if (!list.waitForFinished(6000))
  {
    list.kill();
    _log.emplace_back(
        "ERROR",
        qname + " のcontroller_managerに接続できず、コントローラを"
                "アンロードできませんでした");
    return false;
  }

  QStringList names;
  const QString output = QString::fromUtf8(list.readAllStandardOutput());
  for (const QString &line : output.split('\n', Qt::SkipEmptyParts))
  {
    // Each line is "<name> <type> <state>" (whitespace-separated
    // columns); the controller name is the first token. Skip log lines
    // (e.g. "waiting for service ... to become available"), which start
    // with "[".
    const QString trimmed = line.trimmed();
    if (trimmed.isEmpty() || trimmed.startsWith('['))
      continue;
    const QStringList fields = trimmed.split(' ', Qt::SkipEmptyParts);
    if (!fields.isEmpty())
      names << fields.first();
  }
  if (names.isEmpty())
    return true;

  // Deactivate everything in one call (controllers can depend on each
  // other's interfaces, so switching them all at once is safer than one
  // at a time), then unload each -- unload_controller refuses a
  // controller that is still active.
  //
  // Calls the switch_controller SERVICE directly with `ros2 service call`
  // instead of `ros2 control switch_controllers`: that CLI verb's own
  // --switch-timeout argument is defined without type=float in this ROS 2
  // Jazzy install (ros2controlcli/verb/switch_controllers.py), so passing
  // it crashes with "ValueError: Exceeds the limit ... for integer string
  // conversion" (the string "20" hits `seconds * S_TO_NS`, i.e. Python
  // string-repeats "20" a billion times instead of multiplying a float).
  // That left switch_controllers stuck on its 5s *internal* default
  // (unrelated to the waitForFinished() wall-clock wait below), which
  // controller_manager's own log confirmed was too short for deactivating
  // all 5 EDU controllers at once ("Switch controller timed out after 5
  // seconds!") on a moderately loaded machine -- silently surfacing as
  // "controller release unconfirmed" instead of being fixed. The service
  // call has no such bug and lets a real 20s timeout be requested.
  QString namesYaml;
  for (const QString &controllerName : names)
  {
    if (!namesYaml.isEmpty())
      namesYaml += ", ";
    namesYaml += controllerName;
  }
  const QString switchService =
      "/" + qname + "/controller_manager/switch_controller";
  const QString switchRequest = QString(
      "{deactivate_controllers: [%1], strictness: 1, "
      "activate_asap: false, timeout: {sec: 20, nanosec: 0}}")
      .arg(namesYaml);
  QProcess deactivate;
  deactivate.start("ros2", {"service", "call", switchService,
      "controller_manager_msgs/srv/SwitchController", switchRequest});
  bool ok = deactivate.waitForFinished(25000);
  if (!ok)
    deactivate.kill();
  // `ros2 service call` always exits 0 once the RPC round-trips, even if
  // the response itself reports failure (SwitchController::Response::ok
  // is a field in the reply, not the process exit code) -- the response
  // is only visible in stdout, so that has to be scanned for "ok=True".
  ok = ok && deactivate.exitStatus() == QProcess::NormalExit
          && deactivate.exitCode() == 0
          && QString::fromUtf8(deactivate.readAllStandardOutput())
              .contains("ok=True");
  if (!ok)
  {
    _log.emplace_back(
        "ERROR",
        qname + " のコントローラ非アクティブ化に失敗しました。"
                "アンロードは試みます");
  }

  for (const QString &controllerName : names)
  {
    QProcess unload;
    unload.start("ros2", {"control", "unload_controller",
        controllerName, "-c", controllerManager, "-s"});
    const bool unloadFinished = unload.waitForFinished(4000);
    if (!unloadFinished)
      unload.kill();
    const bool unloadOk = unloadFinished
        && unload.exitStatus() == QProcess::NormalExit
        && unload.exitCode() == 0;
    if (!unloadOk)
    {
      ok = false;
      _log.emplace_back(
          "ERROR",
          qname + " のコントローラ " + controllerName +
              " をアンロードできませんでした");
    }
  }

  if (ok)
  {
    _log.emplace_back(
        "INFO", qname + " のコントローラをアンロードしました");
  }
  else
  {
    _log.emplace_back(
        "WARN",
        qname + " のコントローラを一部アンロードできませんでした。"
                "「表示」で再表示に失敗する場合があります");
  }
  return ok;
}

void EduRobotManager::UnloadRobotControllersAsync(
    const std::string &_robotName, std::function<void(bool)> _onDone)
{
  // Runs the actual `ros2 control` sequence on a detached worker thread
  // so it can block freely (waitForFinished()) without freezing the Qt
  // GUI thread -- see the .hh comment for why this replaced an earlier
  // hand-rolled QProcess/QTimer async chain. QMetaObject::invokeMethod
  // with a context object hops back onto the Qt thread and is a no-op if
  // `this` was destroyed in the meantime (e.g. the panel was unloaded),
  // so no dangling-`this` risk here despite the detached thread outliving
  // this call.
  std::thread([this, _robotName, onDone = std::move(_onDone)]() mutable
  {
    auto log = std::make_shared<std::vector<std::pair<QString, QString>>>();
    const bool ok = UnloadRobotControllersBlocking(_robotName, *log);
    QMetaObject::invokeMethod(this,
        [this, ok, log, onDone = std::move(onDone)]() mutable
    {
      for (const auto &[level, message] : *log)
        this->AppendLog(level, message);
      onDone(ok);
    }, Qt::QueuedConnection);
  }).detach();
}

void EduRobotManager::teleopMove(
    int _robotIndex, double _linear, double _lateral, double _angular)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;
  if (this->worldName.empty())
    return;

  const auto &robot = this->robots.at(_robotIndex);
  if (!robot.visible)
    return;
  this->PublishTwist(robot, _linear, _lateral, _angular);
}

void EduRobotManager::teleopStop(int _robotIndex)
{
  if (_robotIndex < 0 ||
      _robotIndex >= static_cast<int>(this->robots.size()))
    return;

  const auto &robot = this->robots.at(_robotIndex);
  this->PublishTwist(robot, 0.0, 0.0, 0.0);
}

void EduRobotManager::SetStatus(const QString &_status)
{
  this->statusText = _status;
  this->StatusChanged();
}

void EduRobotManager::AppendLog(
    const QString &_level, const QString &_message)
{
  this->logSink.Append("ロボット", _level, _message);
}
}  // namespace edu_gz_gui

GZ_ADD_PLUGIN(
  edu_gz_gui::EduRobotManager,
  gz::gui::Plugin)
