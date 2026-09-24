// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "athena_behavior_tree/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/abstract_logger.h"

namespace athena_behavior_tree
{

class CsvNodeLogger : public BT::StatusChangeLogger
{
public:
  
  CsvNodeLogger(BT::TreeNode * root, const std::string & path,
                std::chrono::steady_clock::time_point start,
                int tree_run)
  : StatusChangeLogger(root), tree_start_(start),
    node_tick_count_(0), tree_run_(tree_run)
  {
    csv_.open(path);  // overwrites existing file
    csv_ << "tree_run,execution_tick,node_name,time_since_start_us,duration_us,status\n";
  }

  ~CsvNodeLogger() override { csv_.close(); }

void callback(BT::Duration timestamp,
                const BT::TreeNode & node,
                BT::NodeStatus prev_status,
                BT::NodeStatus status) override
  {
    (void)timestamp;
    (void)prev_status;

    auto now = std::chrono::steady_clock::now();
    auto since_start_us = std::chrono::duration_cast<std::chrono::microseconds>(
      now - tree_start_).count();

    if (status == BT::NodeStatus::RUNNING) {
      int my_tick = node_tick_count_++;
      start_times_[&node] = now;
      tick_ids_[&node] = my_tick;
      csv_ << tree_run_ << ","
           << my_tick << ","
           << node.name() << ","
           << since_start_us << ","
           << 0 << ","
           << BT::toStr(status) << "\n";
      csv_.flush();
      return;
    }

    // Skip IDLE transitions — they're just sequence resets
    if (status == BT::NodeStatus::IDLE) {
      start_times_.erase(&node);
      tick_ids_.erase(&node);
      return;
    }

    // SUCCESS or FAILURE — reuse the tick from the RUNNING event
    long duration_us = 0;
    auto it = start_times_.find(&node);
    if (it != start_times_.end()) {
      duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
        now - it->second).count();
      start_times_.erase(it);
    }

    int my_tick;
    auto tick_it = tick_ids_.find(&node);
    if (tick_it != tick_ids_.end()) {
      my_tick = tick_it->second;
      tick_ids_.erase(tick_it);
    } else {
      // Node went straight to SUCCESS/FAILURE without a RUNNING event (synchronous node)
      my_tick = node_tick_count_++;
    }

    csv_ << tree_run_ << ","
         << my_tick << ","
         << node.name() << ","
         << since_start_us << ","
         << duration_us << ","
         << BT::toStr(status) << "\n";
    csv_.flush();
  }

  void flush() override { csv_.flush(); }

private:
  std::ofstream csv_;
  std::chrono::steady_clock::time_point tree_start_;
  int node_tick_count_;
  int tree_run_;
  std::map<const BT::TreeNode *, std::chrono::steady_clock::time_point> start_times_;
  std::map<const BT::TreeNode *, int> tick_ids_;
};

BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  // FIXME: the next two line are needed for back-compatibility with BT.CPP 3.8.x
  // Note that the can be removed, once we migrate from BT.CPP 4.5.x to 4.6+
  BT::ReactiveSequence::EnableException(false);
  BT::ReactiveFallback::EnableException(false);
}


BtStatus
BehaviorTreeEngine::run(
  BT::Tree * tree,
  std::function<void()> onLoop,
  std::function<bool()> cancelRequested,
  std::chrono::milliseconds loopTimeout)
{
  rclcpp::WallRate loopRate(loopTimeout);
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  auto tree_start = std::chrono::steady_clock::now();

  tree_run_count_++;

  CsvNodeLogger node_logger(tree->rootNode(), "./bt_nodes.csv", tree_start, tree_run_count_);

  try {
    while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
      if (cancelRequested()) {
        tree->haltTree();
        return BtStatus::CANCELED;
      }
      result = tree->tickOnce();
      onLoop();
      loopRate.sleep();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("BehaviorTreeEngine"),
      "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
    return BtStatus::FAILED;
  }

  auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - tree_start).count();

  RCLCPP_INFO(
    rclcpp::get_logger("BehaviorTreeEngine"),
    "Tree run %d finished: %ld ms, result=%s. CSV saved to bt_nodes.csv",
    tree_run_count_, total_ms, BT::toStr(result).c_str());

  return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

BT::Tree
BehaviorTreeEngine::createTreeFromText(
  const std::string & xml_string,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromText(xml_string, blackboard);
}

BT::Tree
BehaviorTreeEngine::createTreeFromFile(
  const std::string & file_path,
  BT::Blackboard::Ptr blackboard)
{
  return factory_.createTreeFromFile(file_path, blackboard);
}

// In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
void
BehaviorTreeEngine::haltAllActions(BT::TreeNode * root_node)
{
  // this halt signal should propagate through the entire tree.
  root_node->haltNode();

  // but, just in case...
  auto visitor = [](BT::TreeNode * node) {
      if (node->status() == BT::NodeStatus::RUNNING) {
        node->haltNode();
      }
    };
  BT::applyRecursiveVisitor(root_node, visitor);
}

}  // namespace athena_behavior_tree