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

#ifndef HNI_CPP__CHAT_ACTION_SERVER_HPP_
#define HNI_CPP__CHAT_ACTION_SERVER_HPP_

#include <array>
#include <cstring>
#include <functional>
#include <iostream>	 // std::cout
#include <memory>
#include <string>  // std::string, std::stof
#include <thread>
#include <unordered_map>
//#include <vector>

//#include <chrono>
#include "nao_led_interfaces/action/leds_play.hpp"
#include "nao_led_interfaces/msg/led_indexes.hpp"
#include "nao_led_interfaces/msg/led_modes.hpp"
#include "nao_led_interfaces/action/leds_play.hpp"
#include "hni_interfaces/action/joints_play.hpp"
#include "hni_interfaces/action/chat_play.hpp"
#include "hni_interfaces/srv/chat.hpp"
#include "hni_interfaces/srv/text_to_speech.hpp"
#include "hni_cpp/joints_play_action_client.hpp"

#include "hni_cpp/gstt_service_client.hpp"
#include "hni_cpp/gtts_service_client.hpp"
#include "hni_cpp/chat_service_client.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace hni_chat_action_server
{

class ChatActionServer : public rclcpp::Node
{
public:
  explicit ChatActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  virtual ~ChatActionServer();

private:
  // services clients
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gstt_srv_client_;
  rclcpp::Client<hni_interfaces::srv::TextToSpeech>::SharedPtr gtts_srv_client_;
  rclcpp::Client<hni_interfaces::srv::Chat>::SharedPtr chat_srv_client_;

  // client classes
  // std::shared_ptr<hni_led_action_client::LedsPlayActionClient> led_srv_client_;

  // chat play action server
  rclcpp_action::Server<hni_interfaces::action::ChatPlay>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<hni_interfaces::action::ChatPlay>> goal_handle_;
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid,
										 std::shared_ptr<const hni_interfaces::action::ChatPlay::Goal> goal);
  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hni_interfaces::action::ChatPlay>> goal_handle);
  void
  handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hni_interfaces::action::ChatPlay>> goal_handle);
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<hni_interfaces::action::ChatPlay>> goal_handle);

  // joints play action client
  rclcpp_action::Client<hni_interfaces::action::JointsPlay>::SharedPtr joints_act_client_;
  void jointsPlayGoalResponseCallback(
	  const rclcpp_action::ClientGoalHandle<hni_interfaces::action::JointsPlay>::SharedPtr& goal_handle);
  void jointsPlayFeedbackCallback(rclcpp_action::ClientGoalHandle<hni_interfaces::action::JointsPlay>::SharedPtr,
								  const std::shared_ptr<const hni_interfaces::action::JointsPlay::Feedback> feedback);
  void jointsPlayResultCallback(
	  const rclcpp_action::ClientGoalHandle<hni_interfaces::action::JointsPlay>::WrappedResult& result);

  // leds play action client
  rclcpp_action::Client<nao_led_interfaces::action::LedsPlay>::SharedPtr leds_play_act_client_;

  void ledsPlayGoalResponseCallback(
	  const rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr& goal_handle);
  void ledsPlayFeedbackCallback(rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr,
								const std::shared_ptr<const nao_led_interfaces::action::LedsPlay::Feedback> feedback);
  void ledsPlayResultCallback(
	  const rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::WrappedResult& result);

  // parameters
  const double kSecPerWord_;
  const double kForwardParam_;
  std::unordered_map<std::string, std::string> moves_map_;

  /*
  std::shared_ptr<hni_joints_play_action_client::JointsPlayActionClient> joints_play_client_;
  std::shared_ptr<hni_gstt_service_client::GsttServiceClient> gstt_srv_client_;
  std::shared_ptr<hni_gtts_service_client::GttsServiceClient> gtts_srv_client_;
  std::shared_ptr<hni_chat_service_client::ChatServiceClient> chat_srv_client_;
  */

  rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr head_goal_handle_;
  rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr eyes_goal_handle_;
  rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr ears_goal_handle_;
  rclcpp_action::ClientGoalHandle<nao_led_interfaces::action::LedsPlay>::SharedPtr chest_goal_handle_;

  void eyesStatic(bool flag);
  void earsStatic(bool flag);
  void chestStatic(bool flag);
  void headStatic(bool flag);
  void headLoop(bool flag);
  void earsLoop(bool flag);
  void eyesLoop(bool flag);
};

}  // namespace hni_chat_action_server

#endif	// HNI_CPP__CHAT_ACTION_SERVER_HPP_