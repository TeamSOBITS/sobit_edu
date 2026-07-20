import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: root
  // See guider_multifloor_builder's GuiderElevatorControl.qml: the right
  // sidebar width comes from the root item's Layout.minimum* values.
  Layout.minimumWidth: 340
  Layout.minimumHeight: 700
  anchors.fill: parent
  color: "#eef4f2"

  // Selected teleop target (index into EduRobotManager.robotList). This
  // panel only ever manages SOBIT EDU, so there is no robot-type
  // selector -- multiple entries in the list are multiple spawned EDUs.
  property int teleopIndex: robotCombo.currentIndex

  property bool viewSettingsExpanded: false

  function applyViewpoint() {
    EduRobotManager.setViewpoint(
      root.teleopIndex,
      viewpointCombo.currentIndex,
      parseFloat(viewDistanceField.text) || 1.0,
      transitionTimeSlider.value,
      followSpeedSlider.value)
  }

  function currentRobotName() {
    var list = EduRobotManager.robotList
    if (root.teleopIndex < 0 || root.teleopIndex >= list.length)
      return ""
    return list[root.teleopIndex]
  }

  // Checkbox state must be set imperatively: a user click breaks any
  // property binding on `checked`, so bindings would go stale after the
  // first toggle.
  function refreshSensorChecks() {
    var name = currentRobotName()
    var sensors = EduRobotManager.sensorList
    lidarCheck.checked = sensors.indexOf(name + "/lidar") >= 0
    camColorCheck.checked = sensors.indexOf(name + "/color") >= 0
    camDepthCheck.checked = sensors.indexOf(name + "/depth") >= 0
  }

  onTeleopIndexChanged: refreshSensorChecks()

  Flickable {
    anchors.fill: parent
    anchors.margins: 12
    contentHeight: content.height
    clip: true

    Column {
      id: content
      width: parent.width
      spacing: 10

      Row {
        width: parent.width
        spacing: 8
        Rectangle { width: 5; height: 36; radius: 2; color: "#16847c" }
        Column {
          Label { text: "ROBOT"; color: "#126e68"; font.bold: true; font.pixelSize: 12 }
          Label { text: "EDUロボット管理"; color: "#183b37"; font.bold: true; font.pixelSize: 19 }
        }
      }

      Rectangle {
        width: parent.width; height: 32; radius: 6
        color: "#dcefe9"
        Label {
          anchors.centerIn: parent
          text: EduRobotManager.status
          color: "#126e68"
          font.bold: true
        }
      }

      // ── Spawn ──────────────────────────────────────────────
      Label { text: "SOBIT EDUをspawn"; color: "#183b37"; font.bold: true }

      Label {
        width: parent.width
        visible: !EduRobotManager.available()
        text: "⚠ SOBIT EDUが見当たりません：sobit_edu_bringup が" +
              "ワークスペース（src直下）に未導入のためspawnできません。"
        color: "#b3541e"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      Row {
        width: parent.width
        spacing: 8
        Label { text: "名前"; width: 40; anchors.verticalCenter: parent.verticalCenter; color: "#536b67" }
        TextField {
          id: nameField
          width: parent.width - 48
          placeholderText: "model / namespace 名"
          text: EduRobotManager.defaultName()
        }
      }

      GridLayout {
        width: parent.width
        columns: 4
        columnSpacing: 6
        Label { text: "x"; color: "#536b67" }
        // Defaults match gz_minimal.launch.py's old hardcoded spawn pose
        // (rcjo2025_arena's origin area is occupied by furniture; 0,0,0
        // spawns EDU inside a table/shelf).
        TextField { id: xField; text: "-5.5"; Layout.fillWidth: true }
        Label { text: "y"; color: "#536b67" }
        TextField { id: yField; text: "1.5"; Layout.fillWidth: true }
        Label { text: "z"; color: "#536b67" }
        TextField { id: zField; text: "0.05"; Layout.fillWidth: true }
        Label { text: "yaw"; color: "#536b67" }
        TextField { id: yawField; text: "0.0"; Layout.fillWidth: true }
      }

      Button {
        width: parent.width
        text: "Spawn"
        highlighted: true
        enabled: EduRobotManager.available()
        onClicked: {
          EduRobotManager.spawnRobot(
            nameField.text,
            parseFloat(xField.text) || 0.0,
            parseFloat(yField.text) || 0.0,
            parseFloat(zField.text) || 0.0,
            parseFloat(yawField.text) || 0.0)
          nameField.text = EduRobotManager.defaultName()
        }
      }

      Rectangle { width: parent.width; height: 1; color: "#c7d8d4" }

      // ── Robot list / removal ───────────────────────────────
      Label { text: "ロボット一覧"; color: "#183b37"; font.bold: true }

      Repeater {
        model: EduRobotManager.robotList
        delegate: Column {
          width: content.width
          spacing: 2
          property bool hidden: EduRobotManager.hiddenList.indexOf(modelData) >= 0

          Label {
            text: modelData + (hidden ? "　［非表示中］" : "　［表示中］")
            width: parent.width
            elide: Text.ElideRight
            color: hidden ? "#8aa19c" : "#183b37"
            font.bold: !hidden
          }
          Row {
            width: parent.width
            spacing: 6
            Button {
              width: 72
              text: hidden ? "表示" : "非表示"
              onClicked: EduRobotManager.setRobotVisible(index, hidden)
            }
            Button {
              width: 88
              enabled: !hidden
              text: EduRobotManager.rvizList.indexOf(modelData) >= 0 ? "rviz2停止" : "rviz2"
              onClicked: EduRobotManager.toggleRviz(index)
            }
            Button {
              width: 56
              text: "削除"
              onClicked: EduRobotManager.removeRobot(index)
            }
          }
        }
      }

      Label {
        width: parent.width
        visible: EduRobotManager.robotList.length > 0
        text: "「非表示」でノード停止＋モデルを場外へ退避" +
              "（一覧には残ります）。「表示」で同じ場所に復活。"
        color: "#8aa19c"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      Label {
        width: parent.width
        visible: EduRobotManager.robotList.length > 0
        text: "「rviz2」でそのロボット用のrviz2を起動（常に最前面表示）。" +
              "「rviz2停止」で同じウィンドウを終了。"
        color: "#8aa19c"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      Label {
        width: parent.width
        visible: EduRobotManager.robotList.length > 0
        text: "「削除」でノード停止＋モデルを場外へ完全に退避＋一覧から削除" +
              "（元に戻せません）。"
        color: "#8aa19c"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      Label {
        visible: EduRobotManager.robotList.length === 0
        text: "（ロボットなし。上のSpawnボタンで出現します）"
        color: "#8aa19c"
      }

      Rectangle { width: parent.width; height: 1; color: "#c7d8d4" }

      // ── Sensors (runtime toggle) ──────────────────────────
      Label { text: "センサー"; color: "#183b37"; font.bold: true }

      ComboBox {
        id: robotCombo
        width: parent.width
        model: EduRobotManager.robotList
      }

      Label {
        width: parent.width
        text: "ONでセンサーノードが起動しトピック配信が始まります。" +
              "OFFでノードを停止（トピック消滅）。rviz2は再起動せず" +
              "そのまま起動状態を維持しますが、OFF中はトピックが" +
              "無いため対応するDisplayは「No Image」等になります" +
              "（重いので必要な物だけON）。"
        color: "#8aa19c"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      Flow {
        width: parent.width
        spacing: 4
        CheckBox {
          id: lidarCheck
          text: "LiDAR"
          onClicked: EduRobotManager.setSensor(root.teleopIndex, "lidar", checked)
        }
        CheckBox {
          id: camColorCheck
          text: "RGBカメラ"
          onClicked: EduRobotManager.setSensor(root.teleopIndex, "color", checked)
        }
        CheckBox {
          id: camDepthCheck
          text: "深度カメラ"
          onClicked: EduRobotManager.setSensor(root.teleopIndex, "depth", checked)
        }
      }

      Rectangle { width: parent.width; height: 1; color: "#c7d8d4" }

      // ── Camera viewpoint ──────────────────────────────────
      Label { text: "カメラ視点"; color: "#183b37"; font.bold: true }

      Label {
        width: parent.width
        text: "対象：上で選択中のロボット"
        color: "#8aa19c"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }

      ComboBox {
        id: viewpointCombo
        width: parent.width
        enabled: EduRobotManager.robotList.length > 0
        model: [
          "自由視点（通常）",
          "一人称（ロボット視点）",
          "後方追従",
          "前方から",
          "右横から",
          "左横から",
          "俯瞰（真上）",
          "右奥上から（全体俯瞰）",
          "左奥上から（全体俯瞰）"
        ]
        onActivated: root.applyViewpoint()
      }

      Row {
        width: parent.width
        spacing: 8
        Label {
          text: "距離 [m]"
          color: "#536b67"
          font.bold: true
          anchors.verticalCenter: parent.verticalCenter
          width: 70
        }
        TextField {
          id: viewDistanceField
          width: 90
          text: "2.0"
          enabled: viewpointCombo.currentIndex >= 2
          onEditingFinished: {
            if (viewpointCombo.currentIndex > 0)
              root.applyViewpoint()
          }
        }
        Label {
          text: viewpointCombo.currentIndex === 6
            ? "（俯瞰では高さ）" : ""
          color: "#8aa19c"
          font.pixelSize: 11
          anchors.verticalCenter: parent.verticalCenter
        }
      }


      Button {
        width: parent.width
        text: root.viewSettingsExpanded ? "▼ 視点設定" : "▶ 視点設定"
        onClicked: root.viewSettingsExpanded = !root.viewSettingsExpanded
      }

      Column {
        width: parent.width
        spacing: 6
        visible: root.viewSettingsExpanded

        Row {
          width: parent.width
          spacing: 8
          Label {
            text: "切替方法"
            color: "#536b67"
            width: 72
            anchors.verticalCenter: parent.verticalCenter
          }
          ComboBox {
            id: transitionModeCombo
            width: parent.width - 80
            model: ["即時", "標準", "ゆっくり"]
            currentIndex: 1
            onActivated: {
              if (currentIndex === 0)
                transitionTimeSlider.value = 0.0
              else if (currentIndex === 1)
                transitionTimeSlider.value = 0.6
              else
                transitionTimeSlider.value = 1.5
              if (viewpointCombo.currentIndex > 0)
                root.applyViewpoint()
            }
          }
        }

        Row {
          width: parent.width
          spacing: 8
          Label {
            text: "切替時間"
            color: "#536b67"
            width: 72
            anchors.verticalCenter: parent.verticalCenter
          }
          Slider {
            id: transitionTimeSlider
            width: parent.width - 135
            from: 0.0
            to: 2.0
            stepSize: 0.1
            value: 0.6
            enabled: transitionModeCombo.currentIndex !== 0
            onPressedChanged: {
              if (!pressed && viewpointCombo.currentIndex > 0)
                root.applyViewpoint()
            }
          }
          Label {
            text: transitionTimeSlider.value.toFixed(1) + "秒"
            color: "#536b67"
            width: 55
            anchors.verticalCenter: parent.verticalCenter
          }
        }

        Row {
          width: parent.width
          spacing: 8
          Label {
            text: "追従速度"
            color: "#536b67"
            width: 72
            anchors.verticalCenter: parent.verticalCenter
          }
          Label {
            text: "遅い"
            color: "#8aa19c"
            font.pixelSize: 10
            anchors.verticalCenter: parent.verticalCenter
          }
          Slider {
            id: followSpeedSlider
            width: parent.width - 165
            from: 0.05
            to: 1.0
            stepSize: 0.05
            value: 0.35
            onPressedChanged: {
              if (!pressed && viewpointCombo.currentIndex > 0)
                root.applyViewpoint()
            }
          }
          Label {
            text: "速い"
            color: "#8aa19c"
            font.pixelSize: 10
            anchors.verticalCenter: parent.verticalCenter
          }
        }

        Button {
          width: parent.width
          text: "視点設定をデフォルトへ戻す"
          onClicked: {
            transitionModeCombo.currentIndex = 1
            transitionTimeSlider.value = 0.6
            followSpeedSlider.value = 0.35
            if (viewpointCombo.currentIndex > 0)
              root.applyViewpoint()
          }
        }
      }

      Label {
        width: parent.width
        wrapMode: Text.Wrap
        font.pixelSize: 11
        color: "#8aa19c"
        text: "追従中もマウスで視点を微調整できます。一人称は距離入力を" +
              "使いません。実際のセンサー映像はロボット一覧の「rviz2」" +
              "ボタン（LiDAR/RGBカメラ/深度カメラ）で表示できます。"
      }

      Rectangle { width: parent.width; height: 1; color: "#c7d8d4" }

      // ── Teleop ────────────────────────────────────────────
      Label { text: "テレオペ"; color: "#183b37"; font.bold: true }

      Row {
        width: parent.width
        spacing: 8
        Label { text: "速度"; color: "#536b67"; width: 56 }
        Slider {
          id: linSlider
          width: parent.width - 120
          from: 0.1; to: 1.0; value: 0.5
        }
        Label { text: linSlider.value.toFixed(1) + " m/s"; color: "#536b67" }
      }
      Row {
        width: parent.width
        spacing: 8
        Label { text: "旋回"; color: "#536b67"; width: 56 }
        Slider {
          id: angSlider
          width: parent.width - 120
          from: 0.1; to: 1.0; value: 0.5
        }
        Label { text: angSlider.value.toFixed(1) + " rad/s"; color: "#536b67" }
      }

      // Held-button state; the timer republishes at 10 Hz while any
      // direction button is pressed, then sends a stop on release. Runs
      // alongside scripts/edu_teleop_keyboard.py (a separate ROS node,
      // like gz_human_sim's human_teleop_switcher.py) without conflict --
      // both just publish Twist toward the same robot.
      Timer {
        id: teleopTimer
        interval: 100
        repeat: true
        property double lin: 0
        property double lat: 0
        property double ang: 0
        onTriggered: EduRobotManager.teleopMove(
          root.teleopIndex, lin, lat, ang)
      }

      GridLayout {
        width: parent.width
        columns: 3
        columnSpacing: 6
        rowSpacing: 6

        Repeater {
          model: [
            { label: "⟲ 左旋回", lin: 0,  lat: 0,  ang: 1 },
            { label: "▲ 前進",  lin: 1,  lat: 0,  ang: 0 },
            { label: "⟳ 右旋回", lin: 0,  lat: 0,  ang: -1 },
            { label: "◀ 左移動", lin: 0,  lat: 1,  ang: 0 },
            { label: "▼ 後退",  lin: -1, lat: 0,  ang: 0 },
            { label: "右移動 ▶", lin: 0,  lat: -1, ang: 0 }
          ]
          delegate: Button {
            text: modelData.label
            Layout.fillWidth: true
            enabled: EduRobotManager.robotList.length > 0
            onPressed: {
              teleopTimer.lin = modelData.lin * linSlider.value
              teleopTimer.lat = modelData.lat * linSlider.value
              teleopTimer.ang = modelData.ang * angSlider.value
              teleopTimer.start()
              EduRobotManager.teleopMove(
                root.teleopIndex,
                teleopTimer.lin, teleopTimer.lat, teleopTimer.ang)
            }
            onReleased: {
              teleopTimer.stop()
              EduRobotManager.teleopStop(root.teleopIndex)
            }
            onCanceled: {
              teleopTimer.stop()
              EduRobotManager.teleopStop(root.teleopIndex)
            }
          }
        }
      }

      Button {
        width: parent.width
        text: "■ 停止"
        enabled: EduRobotManager.robotList.length > 0
        onClicked: {
          teleopTimer.stop()
          EduRobotManager.teleopStop(root.teleopIndex)
        }
      }

      Label {
        width: parent.width
        wrapMode: Text.Wrap
        font.pixelSize: 11
        color: "#8aa19c"
        text: "キーボードでも操作できます：別ターミナルで " +
              "`ros2 run sobit_edu_gz_gui edu_teleop_keyboard.py` を" +
              "実行してください（i/,/j/l/u/o/m/.で移動、kで停止）。"
      }

      Rectangle { width: parent.width; height: 1; color: "#c7d8d4" }

      Label {
        width: parent.width
        text: "操作の履歴は「EDU Operation Log」パネルで確認できます。"
        color: "#7b928d"
        font.pixelSize: 11
        wrapMode: Text.Wrap
      }
    }
  }

  Connections {
    target: EduRobotManager
    onSensorsChanged: root.refreshSensorChecks()
    onRobotsChanged: root.refreshSensorChecks()
  }
}
