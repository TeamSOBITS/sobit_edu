#include "EduOperationLog.hh"

#include <algorithm>

#include <QClipboard>
#include <QCoreApplication>
#include <QGuiApplication>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include "EduLogSink.hh"

namespace edu_gz_gui
{
// External processes (server systems, CLI) can inject log lines here with
// data = [source, level, message].
static const char kLogTopic[] = "/edu_gz_gui/operation_log";

namespace
{
QString LevelColor(const QString &_level)
{
  if (_level.compare("ERROR", Qt::CaseInsensitive) == 0)
    return "#ff8a80";
  if (_level.compare("WARN", Qt::CaseInsensitive) == 0)
    return "#ffd54f";
  if (_level.compare("OK", Qt::CaseInsensitive) == 0)
    return "#9be8b8";
  return "#d8ebe6";
}
}  // namespace

EduOperationLog::EduOperationLog()
  : gz::gui::Plugin()
{
  this->title = "EDU Operation Log";

  // Known sources are selectable from the start (lines from them may not
  // have arrived yet); unknown sources (e.g. CLI injections) are still
  // appended dynamically in AddEntry.
  this->sources << "すべて" << "ロボット";

  // The bus must be a direct child of the QCoreApplication so
  // EduRobotManager's EduLogSink can find it without sharing any symbols.
  this->bus = new EduOperationLogBus(QCoreApplication::instance());
  this->bus->setObjectName(kOperationLogBusName);

  // appendEntry always runs on the Qt main thread (queued invoke), so a
  // direct connection keeps AddEntry there too.
  QObject::connect(
      this->bus, &EduOperationLogBus::EntryReceived,
      this, &EduOperationLog::AddEntry);
}

EduOperationLog::~EduOperationLog()
{
  // The bus's code lives in this plugin's library: it must not outlive the
  // plugin, so detach it from the application and delete it now.
  if (this->bus)
  {
    this->bus->setParent(nullptr);
    delete this->bus;
    this->bus = nullptr;
  }
}

void EduOperationLog::LoadConfig(const tinyxml2::XMLElement *)
{
  if (!this->node.Subscribe(
      kLogTopic, &EduOperationLog::OnTopicMessage, this))
  {
    gzerr << "[EduOperationLog] failed to subscribe to ["
          << kLogTopic << "]\n";
  }
}

QString EduOperationLog::LogText() const
{
  return this->text;
}

QStringList EduOperationLog::SourceFilters() const
{
  return this->sources;
}

int EduOperationLog::FilterIndex() const
{
  return this->filterIndex;
}

void EduOperationLog::SetFilterIndex(int _index)
{
  const int bounded =
    std::max(0, std::min(_index, this->sources.size() - 1));
  if (bounded == this->filterIndex)
    return;
  this->filterIndex = bounded;
  this->FilterChanged();
  this->RebuildText();
}

QString EduOperationLog::Status() const
{
  if (this->entries.isEmpty())
    return "ログはまだありません";
  if (this->errorCount > 0)
  {
    return QString("%1件（ERROR %2件）")
      .arg(this->entries.size()).arg(this->errorCount);
  }
  return QString("%1件").arg(this->entries.size());
}

void EduOperationLog::clearLog()
{
  // With a specific source selected, only that source's lines are erased;
  // "すべて" erases everything.
  if (this->filterIndex <= 0)
  {
    this->entries.clear();
  }
  else
  {
    const QString source = this->sources.at(this->filterIndex);
    this->entries.erase(
        std::remove_if(
            this->entries.begin(), this->entries.end(),
            [&source](const Entry &_entry)
            { return _entry.source == source; }),
        this->entries.end());
  }

  this->errorCount = 0;
  for (const Entry &entry : this->entries)
  {
    if (entry.level.compare("ERROR", Qt::CaseInsensitive) == 0)
      ++this->errorCount;
  }
  this->RebuildText();
}

void EduOperationLog::copyLog()
{
  QStringList lines;
  for (const Entry &entry : this->entries)
  {
    if (this->Matches(entry))
      lines << FormatPlain(entry);
  }
  if (auto *clipboard = QGuiApplication::clipboard())
    clipboard->setText(lines.join('\n'));
}

void EduOperationLog::AddEntry(
    const QString &_source,
    const QString &_level,
    const QString &_message,
    const QString &_time)
{
  const Entry entry{_source, _level, _message, _time};

  const bool trimmed = this->entries.size() >= kEntryLimit;
  if (trimmed)
  {
    if (this->entries.first().level.compare(
        "ERROR", Qt::CaseInsensitive) == 0)
      --this->errorCount;
    this->entries.removeFirst();
  }
  this->entries.append(entry);
  if (_level.compare("ERROR", Qt::CaseInsensitive) == 0)
    ++this->errorCount;

  if (!_source.isEmpty() && !this->sources.contains(_source))
  {
    this->sources << _source;
    this->SourcesChanged();
  }

  if (trimmed)
  {
    // The oldest line disappeared: the cheap append shortcut below would
    // leave it visible, so rebuild.
    this->RebuildText();
    return;
  }

  if (this->Matches(entry))
  {
    this->text += this->text.isEmpty()
      ? FormatHtml(entry)
      : "<br>" + FormatHtml(entry);
  }
  this->LogChanged();
}

bool EduOperationLog::Matches(const Entry &_entry) const
{
  if (this->filterIndex <= 0 ||
      this->filterIndex >= this->sources.size())
    return true;
  return _entry.source == this->sources.at(this->filterIndex);
}

QString EduOperationLog::FormatHtml(const Entry &_entry)
{
  return QString(
      "<span style=\"color:#8fb0aa\">[%1]</span> "
      "<b style=\"color:#7fd4c9\">[%2]</b> "
      "<span style=\"color:%3\">%4</span>")
    .arg(_entry.time.toHtmlEscaped(),
         _entry.source.toHtmlEscaped(),
         LevelColor(_entry.level),
         _entry.message.toHtmlEscaped());
}

QString EduOperationLog::FormatPlain(const Entry &_entry)
{
  return QString("[%1] [%2] [%3] %4")
    .arg(_entry.time, _entry.source, _entry.level, _entry.message);
}

void EduOperationLog::RebuildText()
{
  QStringList lines;
  for (const Entry &entry : this->entries)
  {
    if (this->Matches(entry))
      lines << FormatHtml(entry);
  }
  this->text = lines.join("<br>");
  this->LogChanged();
}

void EduOperationLog::OnTopicMessage(
    const gz::msgs::StringMsg_V &_message)
{
  const auto field = [&_message](int _index) -> QString
  {
    return _message.data_size() > _index
      ? QString::fromStdString(_message.data(_index))
      : QString();
  };

  const QString source =
    field(0).isEmpty() ? QString("外部") : field(0);
  const QString level = field(1).isEmpty() ? QString("INFO") : field(1);
  const QString message = field(2);
  if (message.isEmpty())
    return;

  // gz-transport thread: hop onto the Qt main thread through the bus.
  QMetaObject::invokeMethod(
      this->bus, "appendEntry", Qt::QueuedConnection,
      Q_ARG(QString, source),
      Q_ARG(QString, level),
      Q_ARG(QString, message),
      Q_ARG(QString, QTime::currentTime().toString("HH:mm:ss")));
}
}  // namespace edu_gz_gui

GZ_ADD_PLUGIN(
  edu_gz_gui::EduOperationLog,
  gz::gui::Plugin)
