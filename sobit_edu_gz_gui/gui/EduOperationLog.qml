import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: root
  Layout.minimumWidth: 340
  Layout.minimumHeight: 420
  anchors.fill: parent
  color: "#eef4f2"

  Column {
    anchors.fill: parent
    anchors.margins: 12
    spacing: 10

    Row {
      width: parent.width
      spacing: 8
      Rectangle { width: 5; height: 36; radius: 2; color: "#16847c" }
      Column {
        Label { text: "LOG"; color: "#126e68"; font.bold: true; font.pixelSize: 12 }
        Label { text: "操作ログ"; color: "#183b37"; font.bold: true; font.pixelSize: 19 }
      }
    }

    Rectangle {
      width: parent.width; height: 32; radius: 6
      color: "#dcefe9"
      Label {
        anchors.centerIn: parent
        text: EduOperationLog.status
        color: "#126e68"
        font.bold: true
      }
    }

    Row {
      width: parent.width
      spacing: 8
      Label {
        text: "表示"
        color: "#536b67"
        font.bold: true
        anchors.verticalCenter: parent.verticalCenter
        width: 44
      }
      ComboBox {
        width: parent.width - 52
        model: EduOperationLog.sourceFilters
        currentIndex: EduOperationLog.filterIndex
        onActivated: EduOperationLog.filterIndex = currentIndex
      }
    }

    Row {
      width: parent.width
      spacing: 8
      Button {
        width: (parent.width - 8) / 2
        text: "コピー"
        onClicked: EduOperationLog.copyLog()
      }
      Button {
        width: (parent.width - 8) / 2
        text: EduOperationLog.filterIndex > 0 ? "選択中のみ消去" : "全て消去"
        onClicked: EduOperationLog.clearLog()
      }
    }

    ScrollView {
      width: parent.width
      height: Math.max(160, parent.height - y)
      clip: true
      TextArea {
        text: EduOperationLog.logText
        textFormat: TextEdit.RichText
        readOnly: true
        selectByMouse: true
        wrapMode: TextEdit.Wrap
        color: "#d8ebe6"
        background: Rectangle { color: "#17312e"; radius: 6 }
        onTextChanged: cursorPosition = length
      }
    }
  }
}
