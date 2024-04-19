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

#ifndef HNI_CPP__GSTT_SERVICE_CLIENT_HPP_
#define HNI_CPP__GSTT_SERVICE_CLIENT_HPP_

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

class GsttServiceClient : public rclcpp::Node
{
public:
  explicit GsttServiceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~GsttServiceClient();

  std::string sendSyncReq();

private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_ptr_;
};

}  // namespace hni_gstt_service_client

#endif  // HNI_CPP__GSTT_SERVICE_CLIENT_HPP_