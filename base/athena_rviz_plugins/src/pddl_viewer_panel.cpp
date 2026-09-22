#include "athena_rviz_plugins/pddl_viewer_panel.hpp"

#include <QDateTime>
#include <QFontDatabase>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QTabWidget>
#include <QVBoxLayout>

#include <exception>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace athena_rviz_plugins
{

PddlViewerPanel::PddlViewerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * tabs = new QTabWidget(this);
  tabs->addTab(createTab(domain_, "/generated_domain"), "Domain");
  tabs->addTab(createTab(problem_, "/generated_problem"), "Problem");

  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(tabs);
  setLayout(layout);
}

PddlViewerPanel::~PddlViewerPanel()
{
  domain_.subscription.reset();
  problem_.subscription.reset();
}

QWidget * PddlViewerPanel::createTab(View & view, const QString & default_topic)
{
  auto * tab = new QWidget();

  view.topic = new QLineEdit(default_topic, tab);
  view.status = new QLabel("Not connected", tab);
  view.text = new QPlainTextEdit(tab);
  view.text->setReadOnly(true);
  view.text->setLineWrapMode(QPlainTextEdit::NoWrap);
  view.text->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  view.text->setPlaceholderText("No message received yet");

  auto * topic_row = new QHBoxLayout();
  topic_row->addWidget(new QLabel("Topic:", tab));
  topic_row->addWidget(view.topic);

  auto * layout = new QVBoxLayout(tab);
  layout->addLayout(topic_row);
  layout->addWidget(view.status);
  layout->addWidget(view.text);

  View * target = &view;
  connect(
    view.topic, &QLineEdit::editingFinished, this,
    [this, target]() {
      subscribe(*target);
      Q_EMIT configChanged();
    });

  return tab;
}

void PddlViewerPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  subscribe(domain_);
  subscribe(problem_);
}

void PddlViewerPanel::subscribe(View & view)
{
  if (!node_) {
    return;
  }

  const std::string topic = view.topic->text().trimmed().toStdString();
  if (view.subscription && topic == view.current_topic) {
    return;
  }

  view.subscription.reset();
  view.current_topic = topic;

  if (topic.empty()) {
    view.status->setText("No topic set");
    return;
  }

  View * target = &view;
  try {
    view.subscription = node_->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10),
      [this, target](std_msgs::msg::String::ConstSharedPtr msg) {
        const QString text = QString::fromStdString(msg->data);
        QMetaObject::invokeMethod(
          this, [this, target, text]() {showText(*target, text);}, Qt::QueuedConnection);
      });
    view.status->setText(QString("Waiting for messages on %1").arg(QString::fromStdString(topic)));
  } catch (const std::exception & e) {
    view.status->setText(QString("Error: %1").arg(e.what()));
  }
}

void PddlViewerPanel::showText(View & view, const QString & text)
{
  const int scroll = view.text->verticalScrollBar()->value();
  view.text->setPlainText(text);
  view.text->verticalScrollBar()->setValue(scroll);
  view.status->setText(
    QString("Received at %1, %2 lines")
    .arg(QDateTime::currentDateTime().toString("HH:mm:ss"))
    .arg(text.count('\n') + 1));
}

void PddlViewerPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("domain_topic", domain_.topic->text());
  config.mapSetValue("problem_topic", problem_.topic->text());
}

void PddlViewerPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("domain_topic", &topic)) {
    domain_.topic->setText(topic);
  }
  if (config.mapGetString("problem_topic", &topic)) {
    problem_.topic->setText(topic);
  }
  subscribe(domain_);
  subscribe(problem_);
}

}  // namespace athena_rviz_plugins

PLUGINLIB_EXPORT_CLASS(athena_rviz_plugins::PddlViewerPanel, rviz_common::Panel)
