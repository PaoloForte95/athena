#ifndef ATHENA_RVIZ_PLUGINS__PDDL_VIEWER_PANEL_HPP_
#define ATHENA_RVIZ_PLUGINS__PDDL_VIEWER_PANEL_HPP_

#include <QString>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QWidget;

namespace athena_rviz_plugins
{

class PddlViewerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PddlViewerPanel(QWidget * parent = nullptr);
  ~PddlViewerPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  struct View
  {
    QLineEdit * topic = nullptr;
    QLabel * status = nullptr;
    QPlainTextEdit * text = nullptr;
    std::string current_topic;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  };

  QWidget * createTab(View & view, const QString & default_topic);
  void subscribe(View & view);
  void showText(View & view, const QString & text);

  rclcpp::Node::SharedPtr node_;
  View domain_;
  View problem_;
};

}  // namespace athena_rviz_plugins

#endif  // ATHENA_RVIZ_PLUGINS__PDDL_VIEWER_PANEL_HPP_
