#ifndef SOBIT_EDU_GZ_GUI_EDULOGSINK_HH_
#define SOBIT_EDU_GZ_GUI_EDULOGSINK_HH_

#include <QCoreApplication>
#include <QList>
#include <QMetaObject>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QTime>

namespace edu_gz_gui
{
/// \brief Object name of the process-wide log bus the EduOperationLog
/// panel parents to the QApplication; publisher plugins locate it by this
/// name, so no symbols are shared between the plugin libraries.
static constexpr const char kOperationLogBusName[] = "EduOperationLogBus";

/// \brief Forwards EduRobotManager's operation-log lines to the
/// EduOperationLog panel.
///
/// Lines are timestamped immediately and buffered until the log panel (and
/// with it the bus object) exists, so messages emitted while the GUI is
/// still loading plugins are not lost. Delivery uses a string-based queued
/// QMetaObject::invokeMethod, which keeps the publishing plugin decoupled
/// from the panel's class definition.
///
/// Append() must be called from the Qt main thread only (findChild walks
/// the QApplication's children); every existing caller already marshals
/// gz-transport callbacks onto the Qt thread before logging.
class EduLogSink
{
  public: void Append(
      const QString &_source,
      const QString &_level,
      const QString &_message)
  {
    const Entry entry{
      _source,
      _level,
      _message,
      QTime::currentTime().toString("HH:mm:ss"),
    };

    if (!this->bus)
    {
      auto *app = QCoreApplication::instance();
      if (app)
      {
        this->bus = app->findChild<QObject *>(
            kOperationLogBusName, Qt::FindDirectChildrenOnly);
      }
    }

    if (!this->bus)
    {
      this->pending.append(entry);
      if (this->pending.size() > kPendingLimit)
        this->pending.removeFirst();
      return;
    }

    for (const Entry &buffered : this->pending)
      this->Invoke(buffered);
    this->pending.clear();

    this->Invoke(entry);
  }

  private: struct Entry
  {
    QString source;
    QString level;
    QString message;
    QString time;
  };

  private: void Invoke(const Entry &_entry)
  {
    QMetaObject::invokeMethod(
        this->bus, "appendEntry", Qt::QueuedConnection,
        Q_ARG(QString, _entry.source),
        Q_ARG(QString, _entry.level),
        Q_ARG(QString, _entry.message),
        Q_ARG(QString, _entry.time));
  }

  private: static constexpr int kPendingLimit = 100;

  private: QPointer<QObject> bus;

  private: QList<Entry> pending;
};
}  // namespace edu_gz_gui

#endif  // SOBIT_EDU_GZ_GUI_EDULOGSINK_HH_
