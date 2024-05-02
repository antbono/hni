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

#ifndef HNI_CPP__HEAD_TRACK_ACTION_SERVER_HPP_
#define HNI_CPP__HEAD_TRACK_ACTION_SERVER_HPP_

#include <chrono>
#include <functional>
#include <iostream>
#include <iostream>  // std::cout
#include <map>
#include <memory>
#include <queue>   // std::queue
#include <string>  // std::string, std::stof
#include <thread>
#include <vector>

#include "hni_interfaces/action/video_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "hni_interfaces/action/videoTracker_feedback_message.hpp" //?

#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace hni_head_track_action_server
{

using HeadTrack = hni_interfaces::action::VideoTracker;
using GoalHandleHeadTrack = rclcpp_action::ServerGoalHandle<HeadTrack>;
using ObjTrack = hni_interfaces::action::VideoTracker;
using GoalHandleObjTrack = rclcpp_action::ClientGoalHandle<ObjTrack>;

class HeadTrackActionServer : public rclcpp::Node
{
public:
  explicit HeadTrackActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~HeadTrackActionServer();

private:
  // objects
  rclcpp_action::Server<HeadTrack>::SharedPtr action_server_;
  rclcpp_action::Client<ObjTrack>::SharedPtr client_ptr_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr jpos_sub_;

  rclcpp::Subscription<hni_interfaces::action::VideoTracker_FeedbackMessage>::SharedPtr
    obj_pos_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  nao_lola_command_msgs::msg::JointIndexes joint_indexes_msg_;

  std::queue<float> x_track_;
  std::queue<float> y_track_;
  float last_yaw_;
  float last_pitch_;
  const double kSecToHeadReset_;
  const uint8_t kTrackMaxSize_;
  const float kHeadWidthStep_;   // = 0.36;
  const float kHeadHeightStep_;  // = 0.25;
  const float kVerResolution_;   // = 480; // y
  const float kHorResolution_;   // = 640;  // x

  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;

  bool fileSuccessfullyRead_;
  std::map<std::vector<float>, std::string> head_pos_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void jposCallback(const nao_lola_sensor_msgs::msg::JointPositions & joints);
  void sendGoal();
  void goalResponseCallback(const GoalHandleObjTrack::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleObjTrack::SharedPtr, const std::shared_ptr<const ObjTrack::Feedback> feedback);
  void resultCallback(const GoalHandleObjTrack::WrappedResult & result);
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const HeadTrack::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleHeadTrack> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleHeadTrack> goal_handle);
  void execute(const std::shared_ptr<GoalHandleHeadTrack> goal_handle);

};  // class

}  // namespace hni_head_track_action_server

#endif  // HNI_CPP__HEAD_TRACK_ACTION_SERVER_HPP_