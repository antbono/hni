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

#include "hni_cpp/gtts_service_client.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hni_gtts_service_client
{

GttsServiceClient::GttsServiceClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("gtts_service_client_node", options)
{
  this->client_ptr_ = this->create_client<hni_interfaces::srv::TextToSpeech>("gtts_service");

  RCLCPP_INFO(this->get_logger(), "GttsServiceClient initialized");
}

GttsServiceClient::~GttsServiceClient() {}

void GttsServiceClient::sendSyncReq(std::string & text_to_speak)
{
  using namespace std::chrono_literals;

  while (!client_ptr_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for GTTS service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GTTS service not available, waiting again...");
  }

  // speaking
  auto gtts_request = std::make_shared<hni_interfaces::srv::TextToSpeech::Request>();
  gtts_request->text = text_to_speak;
  auto gtts_result = client_ptr_->async_send_request(gtts_request);
  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), gtts_result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "GTTS request completed: %d", gtts_result.get()->success);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call gtts_service");
    return;
  }
}

}  // namespace hni_gtts_service_client

RCLCPP_COMPONENTS_REGISTER_NODE(hni_gtts_service_client::GttsServiceClient)