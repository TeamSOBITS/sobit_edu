#ifndef SOBIT_EDU_GZ_GUI_EDUOPERATIONLOG_HH_
#define SOBIT_EDU_GZ_GUI_EDUOPERATIONLOG_HH_

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/transport/Node.hh>

namespace edu_gz_gui
{
/// \brief Process-wide receiver for operation-log lines.
///
/// The EduOperationLog panel parents one instance to the QApplication
/// under the name kOperationLogBusName (EduLogSink.hh); EduRobotManager
/// finds it there and calls appendEntry() via a queued string-based
/// invoke, so appendEntry always runs on the Qt main thread.
class EduOperationLogBus : public QObject
{
  Q_OBJECT

  public: explicit EduOperationLogBus(QObject *_parent = nullptr)
    : QObject(_parent)
  {
  }

  public: Q_INVOKABLE void appendEntry(
      const QString &_source,
      const QString &_level,
      const QString &_message,
      const QString &_time)
  {
    emit this->EntryReceived(_source, _level, _message, _time);
  }

  signals: void EntryReceived(
      const QString &_source,
      const QString &_level,
      const QString &_message,
      const QString &_time);
};

/// \brief Sidebar panel showing EduRobotManager's operation log.
///
/// Sources:
///  - in-process: EduRobotManager publishes through EduLogSink to the bus
///    object above;
///  - external: gz.msgs.StringMsg_V on /edu_gz_gui/operation_log with
///    data = [source, level, message], so server-side systems or the CLI
///    can inject lines too.
///
/// Entries are kept in a bounded list, filterable by source; the visible
/// slice is exposed to QML as rich text with per-level colors.
class EduOperationLog : public gz::gui::Plugin
{
  Q_OBJECT

  /// \brief Rich-text log of the currently filtered entries.
  Q_PROPERTY(QString logText READ LogText NOTIFY LogChanged)

  /// \brief "すべて" plus every source seen so far.
  Q_PROPERTY(
    QStringList sourceFilters
    READ SourceFilters
    NOTIFY SourcesChanged)

  /// \brief Index into sourceFilters (0 = show everything).
  Q_PROPERTY(
    int filterIndex
    READ FilterIndex
    WRITE SetFilterIndex
    NOTIFY FilterChanged)

  Q_PROPERTY(QString status READ Status NOTIFY LogChanged)

  public: EduOperationLog();

  public: ~EduOperationLog() override;

  protected: void LoadConfig(
      const tinyxml2::XMLElement *_pluginElem) override;

  public: QString LogText() const;

  public: QStringList SourceFilters() const;

  public: int FilterIndex() const;

  public: void SetFilterIndex(int _index);

  public: QString Status() const;

  public: Q_INVOKABLE void clearLog();

  /// \brief Copy the filtered entries to the clipboard as plain text.
  public: Q_INVOKABLE void copyLog();

  signals: void LogChanged();

  signals: void SourcesChanged();

  signals: void FilterChanged();

  private slots: void AddEntry(
      const QString &_source,
      const QString &_level,
      const QString &_message,
      const QString &_time);

  private: struct Entry
  {
    QString source;
    QString level;
    QString message;
    QString time;
  };

  private: bool Matches(const Entry &_entry) const;

  private: static QString FormatHtml(const Entry &_entry);

  private: static QString FormatPlain(const Entry &_entry);

  /// \brief Rebuild the rich-text view from scratch (filter change,
  /// clear, trim); plain appends avoid this.
  private: void RebuildText();

  private: void OnTopicMessage(const gz::msgs::StringMsg_V &_message);

  private: static constexpr int kEntryLimit = 300;

  private: EduOperationLogBus *bus{nullptr};

  private: QVector<Entry> entries;

  private: QStringList sources;

  private: int filterIndex{0};

  private: int errorCount{0};

  private: QString text;

  private: gz::transport::Node node;
};
}  // namespace edu_gz_gui

#endif  // SOBIT_EDU_GZ_GUI_EDUOPERATIONLOG_HH_
