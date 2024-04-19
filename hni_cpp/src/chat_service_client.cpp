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

#include "hni_cpp/chat_service_client.hpp"

#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hni_chat_service_client
{

ChatServiceClient::ChatServiceClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("chat_service_client_node", options)
{
  this->client_ptr_ = this->create_client<hni_interfaces::srv::Chat>("chatGPT_service");

  RCLCPP_INFO(this->get_logger(), "ChatServiceClient initialized");
}

ChatServiceClient::~ChatServiceClient() {}

std::string ChatServiceClient::sendSyncReq(std::string & phrase)
{
  using namespace std::chrono_literals;

  while (!client_ptr_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for chat service. Exiting.");
      return "ERROR";
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "chat service not available, waiting again...");
  }

  // chatgpt
  std::string chatgpt_answer;
  auto chat_request = std::make_shared<hni_interfaces::srv::Chat::Request>();
  chat_request->question = phrase;
  auto chat_result = client_ptr_->async_send_request(chat_request);

  RCLCPP_DEBUG(this->get_logger(), "wait chatGPT answer..");
  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), chat_result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    chatgpt_answer = chat_result.get()->answer;
    RCLCPP_INFO(this->get_logger(), ("chatGPT answer: " + chatgpt_answer).c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call chat_service");
    return "ERROR";
  }
  RCLCPP_DEBUG(this->get_logger(), "chat request completed");
  return chatgpt_answer;
}

}  // namespace hni_chat_service_client

RCLCPP_COMPONENTS_REGISTER_NODE(hni_chat_service_client::ChatServiceClient)