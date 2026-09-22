#include "athena_rviz_plugins/execution_plan_panel.hpp"

#include <QBrush>
#include <QColor>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <exception>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace athena_rviz_plugins
{

namespace
{
using Event = athena_msgs::msg::Event;

const QColor kRunningColor(255, 183, 77);
const QColor kDoneColor(129, 199, 132);
const QColor kFailedColor(229, 115, 115);
}  // namespace

ExecutionPlanPanel::ExecutionPlanPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  plan_topic_edit_ = new QLineEdit("/dispatched_plan", this);
  event_topic_edit_ = new QLineEdit("/plan_actions", this);
  summary_ = new QLabel("Not connected", this);

  auto * topics = new QFormLayout();
  topics->addRow("Plan topic:", plan_topic_edit_);
  topics->addRow("Event topic:", event_topic_edit_);

  actions_table_ = createTable({"ID", "Robot", "Name", "Object", "Parents", "Status"});
  methods_table_ = createTable({"ID", "Robot", "Name", "Parents", "Subtasks", "Status"});

  tabs_ = new QTabWidget(this);
  tabs_->addTab(actions_table_, "Actions");
  tabs_->addTab(methods_table_, "Methods");

  auto * layout = new QVBoxLayout(this);
  layout->addLayout(topics);
  layout->addWidget(summary_);
  layout->addWidget(tabs_);
  setLayout(layout);

  auto on_topic_changed = [this]() {
      subscribe();
      Q_EMIT configChanged();
    };
  connect(plan_topic_edit_, &QLineEdit::editingFinished, this, on_topic_changed);
  connect(event_topic_edit_, &QLineEdit::editingFinished, this, on_topic_changed);
}

ExecutionPlanPanel::~ExecutionPlanPanel()
{
  plan_sub_.reset();
  event_sub_.reset();
}

QTableWidget * ExecutionPlanPanel::createTable(const QStringList & headers)
{
  auto * table = new QTableWidget(0, headers.size(), this);
  table->setHorizontalHeaderLabels(headers);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionBehavior(QAbstractItemView::SelectRows);
  table->verticalHeader()->setVisible(false);
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table->horizontalHeader()->setStretchLastSection(true);
  return table;
}

void ExecutionPlanPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  subscribe();
}

void ExecutionPlanPanel::subscribe()
{
  if (!node_) {
    return;
  }

  const std::string plan_topic = plan_topic_edit_->text().trimmed().toStdString();
  const std::string event_topic = event_topic_edit_->text().trimmed().toStdString();

  try {
    if (!plan_sub_ || plan_topic != plan_topic_) {
      plan_sub_.reset();
      plan_topic_ = plan_topic;
      if (!plan_topic.empty()) {
        plan_sub_ = node_->create_subscription<athena_msgs::msg::Plan>(
          plan_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          [this](athena_msgs::msg::Plan::ConstSharedPtr msg) {
            QMetaObject::invokeMethod(
              this, [this, msg]() {onPlan(*msg);}, Qt::QueuedConnection);
          });
      }
    }

    if (!event_sub_ || event_topic != event_topic_) {
      event_sub_.reset();
      event_topic_ = event_topic;
      if (!event_topic.empty()) {
        event_sub_ = node_->create_subscription<Event>(
          event_topic, rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local().reliable(),
          [this](Event::ConstSharedPtr msg) {
            QMetaObject::invokeMethod(
              this, [this, msg]() {onEvent(*msg);}, Qt::QueuedConnection);
          });
      }
    }
    updateSummary();
  } catch (const std::exception & e) {
    summary_->setText(QString("Error: %1").arg(e.what()));
  }
}

void ExecutionPlanPanel::onPlan(const athena_msgs::msg::Plan & plan)
{
  if (has_plan_ && plan == plan_) {
    return;
  }
  if (has_plan_) {
    states_.clear();
  }
  plan_ = plan;
  has_plan_ = true;
  rebuildTables();
}

void ExecutionPlanPanel::onEvent(const Event & event)
{
  State state;
  if (event.status == Event::RUNNING) {
    state = State::Running;
  } else if (event.status == Event::SUCCESS) {
    state = State::Done;
  } else if (event.status == Event::FAILURE) {
    state = State::Failed;
  } else {
    return;
  }

  const Key key{static_cast<uint8_t>(event.kind), static_cast<int32_t>(event.id)};
  states_[key] = state;

  const auto row = rows_.find(key);
  if (row != rows_.end()) {
    paintRow(row->second, state);
  }
  updateSummary();
}

void ExecutionPlanPanel::rebuildTables()
{
  rows_.clear();

  actions_table_->setRowCount(0);
  actions_table_->setRowCount(static_cast<int>(plan_.actions.size()));
  int index = 0;
  for (const auto & action : plan_.actions) {
    fillRow(
      actions_table_, index,
      {QString::number(action.action_id),
        QString::fromStdString(action.robot),
        QString::fromStdString(action.name),
        QString::fromStdString(action.object),
        joinIds(action.parents)});
    rows_[{Event::ACTION, action.action_id}] = Row{actions_table_, index};
    ++index;
  }

  methods_table_->setRowCount(0);
  methods_table_->setRowCount(static_cast<int>(plan_.methods.size()));
  index = 0;
  for (const auto & method : plan_.methods) {
    fillRow(
      methods_table_, index,
      {QString::number(method.id),
        QString::fromStdString(method.robot),
        QString::fromStdString(method.name),
        joinIds(method.parents),
        joinIds(method.substasks)});
    rows_[{Event::METHOD, method.id}] = Row{methods_table_, index};
    ++index;
  }

  tabs_->setTabText(0, QString("Actions (%1)").arg(plan_.actions.size()));
  tabs_->setTabText(1, QString("Methods (%1)").arg(plan_.methods.size()));
  if (plan_.actions.empty() && !plan_.methods.empty()) {
    tabs_->setCurrentIndex(1);
  }

  for (const auto & [key, row] : rows_) {
    paintRow(row, stateOf(key));
  }
  updateSummary();
}

void ExecutionPlanPanel::fillRow(QTableWidget * table, int row, const QStringList & values)
{
  for (int column = 0; column < values.size(); ++column) {
    table->setItem(row, column, new QTableWidgetItem(values[column]));
  }
  table->setItem(row, table->columnCount() - 1, new QTableWidgetItem());
}

void ExecutionPlanPanel::paintRow(const Row & row, State state)
{
  QString text;
  QColor color;
  switch (state) {
    case State::Running:
      text = "Running";
      color = kRunningColor;
      break;
    case State::Done:
      text = "Done";
      color = kDoneColor;
      break;
    case State::Failed:
      text = "Failed";
      color = kFailedColor;
      break;
    case State::Pending:
      text = "Pending";
      break;
  }

  QTableWidget * table = row.table;
  table->item(row.index, table->columnCount() - 1)->setText(text);

  for (int column = 0; column < table->columnCount(); ++column) {
    QTableWidgetItem * item = table->item(row.index, column);
    if (!item) {
      continue;
    }
    if (color.isValid()) {
      item->setBackground(QBrush(color));
      item->setForeground(QBrush(Qt::black));
    } else {
      item->setData(Qt::BackgroundRole, QVariant());
      item->setData(Qt::ForegroundRole, QVariant());
    }
  }
}

void ExecutionPlanPanel::updateSummary()
{
  if (!has_plan_) {
    summary_->setText(
      QString("Waiting for a plan on %1").arg(QString::fromStdString(plan_topic_)));
    return;
  }

  int running = 0;
  int done = 0;
  int failed = 0;
  int pending = 0;
  for (const auto & entry : rows_) {
    switch (stateOf(entry.first)) {
      case State::Running: ++running; break;
      case State::Done: ++done; break;
      case State::Failed: ++failed; break;
      case State::Pending: ++pending; break;
    }
  }

  summary_->setText(
    QString("Running: %1    Done: %2    Failed: %3    Pending: %4")
    .arg(running).arg(done).arg(failed).arg(pending));
}

ExecutionPlanPanel::State ExecutionPlanPanel::stateOf(const Key & key) const
{
  const auto it = states_.find(key);
  return it == states_.end() ? State::Pending : it->second;
}

QString ExecutionPlanPanel::joinIds(const std::vector<int32_t> & ids) const
{
  QStringList parts;
  for (const int32_t id : ids) {
    parts << QString::number(id);
  }
  return parts.join(", ");
}

void ExecutionPlanPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("plan_topic", plan_topic_edit_->text());
  config.mapSetValue("event_topic", event_topic_edit_->text());
}

void ExecutionPlanPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("plan_topic", &topic)) {
    plan_topic_edit_->setText(topic);
  }
  if (config.mapGetString("event_topic", &topic)) {
    event_topic_edit_->setText(topic);
  }
  subscribe();
}

}  // namespace athena_rviz_plugins

PLUGINLIB_EXPORT_CLASS(athena_rviz_plugins::ExecutionPlanPanel, rviz_common::Panel)
