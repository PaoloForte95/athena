#ifndef ATHENA_RVIZ_PLUGINS__EXECUTION_PLAN_PANEL_HPP_
#define ATHENA_RVIZ_PLUGINS__EXECUTION_PLAN_PANEL_HPP_

#include <QString>
#include <QStringList>

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <athena_msgs/msg/event.hpp>
#include <athena_msgs/msg/plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>

class QLabel;
class QLineEdit;
class QTabWidget;
class QTableWidget;

namespace athena_rviz_plugins
{

class ExecutionPlanPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExecutionPlanPanel(QWidget * parent = nullptr);
  ~ExecutionPlanPanel() override;

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  enum class State
  {
    Pending,
    Running,
    Done,
    Failed
  };

  using Key = std::pair<uint8_t, int32_t>;

  struct Row
  {
    QTableWidget * table = nullptr;
    int index = 0;
  };

  QTableWidget * createTable(const QStringList & headers);
  void subscribe();
  void onPlan(const athena_msgs::msg::Plan & plan);
  void onEvent(const athena_msgs::msg::Event & event);
  void rebuildTables();
  void fillRow(QTableWidget * table, int row, const QStringList & values);
  void paintRow(const Row & row, State state);
  void updateSummary();
  State stateOf(const Key & key) const;
  QString joinIds(const std::vector<int32_t> & ids) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<athena_msgs::msg::Plan>::SharedPtr plan_sub_;
  rclcpp::Subscription<athena_msgs::msg::Event>::SharedPtr event_sub_;
  std::string plan_topic_;
  std::string event_topic_;

  QLineEdit * plan_topic_edit_ = nullptr;
  QLineEdit * event_topic_edit_ = nullptr;
  QLabel * summary_ = nullptr;
  QTabWidget * tabs_ = nullptr;
  QTableWidget * actions_table_ = nullptr;
  QTableWidget * methods_table_ = nullptr;

  athena_msgs::msg::Plan plan_;
  bool has_plan_ = false;
  std::map<Key, State> states_;
  std::map<Key, Row> rows_;
};

}  // namespace athena_rviz_plugins

#endif  // ATHENA_RVIZ_PLUGINS__EXECUTION_PLAN_PANEL_HPP_
