// Copyright 2024 Antonio Bono
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

#ifndef HNI_CPP__JOINTS_PLAY_ACTION_SERVER_HPP_
#define HNI_CPP__JOINTS_PLAY_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <thread>         // std::thread, std::this_thread::yield
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>     // std::string, std::stof std::c_str

#include <iostream>   // std::cout

#include "hni_interfaces/action/joints_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"

namespace hni_joints_play_action_server {

using JointsPlay = hni_interfaces::action::JointsPlay;
using GoalHandleJointsPlay = rclcpp_action::ServerGoalHandle<JointsPlay>;

class JointsPlayActionServer : public rclcpp::Node {

  public:
	explicit JointsPlayActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
	virtual ~JointsPlayActionServer();

  private:

  rclcpp_action::Server<JointsPlay>::SharedPtr action_server_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;

  std::ifstream ifs_;
  std::vector<float> recorded_joints_;
  
  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;

  bool fileSuccessfullyRead_;
  std::atomic<bool> free_;


  rclcpp_action::GoalResponse handleGoal( const rclcpp_action::GoalUUID & uuid,
  	std::shared_ptr<const JointsPlay::Goal> goal);
  rclcpp_action::CancelResponse handleCancel( const std::shared_ptr<GoalHandleJointsPlay> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleJointsPlay> goal_handle);
  void execute(const std::shared_ptr<GoalHandleJointsPlay> goal_handle);


};

} // namespace hni_joints_play_action_server


#endif  // HNI_CPP__JOINTS_PLAY_ACTION_SERVER_HPP_