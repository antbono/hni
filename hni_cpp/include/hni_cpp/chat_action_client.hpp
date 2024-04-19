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

#ifndef HNI_MOVES__CHAT_ACTION_CLIENT_HPP_
#define HNI_MOVES__CHAT_ACTION_CLIENT_HPP_

#include <array>
#include <cstring>
#include <functional>
#include <iostream>  // std::cout
#include <memory>
#include <string>  // std::string, std::stof
#include <thread>
#include <unordered_map>
//#include <vector>

//#include <chrono>
#include "hni_interfaces/action/chat_play.hpp"
#include "hni_interfaces/action/joints_play.hpp"
#include "hni_interfaces/srv/chat.hpp"
#include "hni_interfaces/srv/text_to_speech.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace hni_chat_action_client
{

class ChatActionClient : public rclcpp::Node
{
public:
  explicit ChatActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~ChatActionClient();

private:
  void sendAsyncGoal();
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::SharedPtr &
      goal_handle);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::SharedPtr,
    const std::shared_ptr<const hni_interfaces::action::ChatPlay::Feedback> feedback);
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::WrappedResult &
      result);

  rclcpp_action::Client<hni_interfaces::action::ChatPlay>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

};  // ChatActionClient

}  // namespace hni_chat_action_client

#endif  // HNI_MOVES__CHAT_ACTION_CLIENT_HPP_