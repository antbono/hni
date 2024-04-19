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

#ifndef HNI_CPP__GTTS_SERVICE_CLIENT_HPP_
#define HNI_CPP__GTTS_SERVICE_CLIENT_HPP_

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "hni_interfaces/srv/text_to_speech.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace hni_gtts_service_client
{

class GttsServiceClient : public rclcpp::Node
{
public:
  explicit GttsServiceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~GttsServiceClient();

  void sendSyncReq(std::string & text_to_speak);

private:
  rclcpp::Client<hni_interfaces::srv::TextToSpeech>::SharedPtr client_ptr_;
};

}  // namespace hni_gtts_service_client

#endif  // HNI_CPP__GTTS_SERVICE_CLIENT_HPP_