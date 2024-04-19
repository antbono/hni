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

#include "hni_cpp/gstt_service_client.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace hni_gstt_service_client
{

GsttServiceClient::GsttServiceClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("gstt_service_client_node", options)
{
  this->client_ptr_ = this->create_client<std_srvs::srv::SetBool>("gstt_service");

  RCLCPP_INFO(this->get_logger(), "GsttServiceClient initialized");
}

GsttServiceClient::~GsttServiceClient() {}

std::string GsttServiceClient::sendSyncReq()
{
  using namespace std::chrono_literals;

  while (!client_ptr_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for GSTT service. Exiting.");
      return "ERROR";
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GSTT service not available, waiting again...");
  }

  std::string recognized_speach;
  auto gstt_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  gstt_request->data = true;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GSTT sending request...");
  auto gstt_result = client_ptr_->async_send_request(gstt_request);
  // Wait for the result.
  if (
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), gstt_result) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inside spin_until_future_complete");
    // if ( rclcpp::spin_until_future_complete(this, gstt_result) == rclcpp::FutureReturnCode::SUCCESS ) {
    recognized_speach = gstt_result.get()->message;
    RCLCPP_INFO(this->get_logger(), ("Recognized speach: " + recognized_speach).c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call gstt_service");
    return "ERROR";
  }

  return recognized_speach;
}

}  // namespace hni_gstt_service_client

RCLCPP_COMPONENTS_REGISTER_NODE(hni_gstt_service_client::GsttServiceClient)