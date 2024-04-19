#include "hni_cpp/joints_play_action_client.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "hni_interfaces/action/joints_play.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fs = boost::filesystem;

namespace hni_joints_play_action_client
{

JointsPlayActionClient::JointsPlayActionClient(const rclcpp::NodeOptions & options)
: rclcpp::Node("joints_play_action_server_node", options)
{
  this->client_ptr_ = rclcpp_action::create_client<JointsPlay>(this, "joints_play");

  /*this->timer_ = this->create_wall_timer(
                   std::chrono::milliseconds(500),
                   std::bind(&JointsPlayActionClient::sendGoal, this));*/

  this->declare_parameter<std::string>("file", getDefaultFullFilePath());

  RCLCPP_INFO(this->get_logger(), "JointsPlayActionClient initialized");
}

JointsPlayActionClient::~JointsPlayActionClient() {}

/*
void JointsPlayActionClient::sendGoal()  {
  using namespace std::placeholders;

  //this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = JointsPlay::Goal();

  std::string filePath;
  this->get_parameter("file", filePath);

  goal_msg.path = filePath;

  auto send_goal_options = rclcpp_action::Client<JointsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&JointsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&JointsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&JointsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), ("Sending goal: " + filePath).c_str() );

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}*/

void JointsPlayActionClient::sendAsyncGoal(std::string & action_path)
{
  using namespace std::placeholders;

  // this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = JointsPlay::Goal();

  goal_msg.path = action_path;

  auto send_goal_options = rclcpp_action::Client<JointsPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    std::bind(&JointsPlayActionClient::goalResponseCallback, this, _1);

  send_goal_options.feedback_callback =
    std::bind(&JointsPlayActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback = std::bind(&JointsPlayActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), ("Sending goal: " + action_path).c_str());

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

std::string JointsPlayActionClient::getDefaultFullFilePath()
{
  /*
  RCLCPP_INFO(this->get_logger(),"getDefaultFullFilePath ");
  std::string file = "moves/hello.txt";
  std::string package_share_directory = ament_index_cpp::get_package_share_directory(
                                          "hni_cpp");
  // e.g. /home/nao/rolling_ws/install/hni_cpp/share/hni_cpp

  RCLCPP_INFO(this->get_logger(),("package_share_directory: "+package_share_directory).c_str());
  fs::path dir_path(package_share_directory);
  RCLCPP_INFO(this->get_logger(),("dir path "+dir_path.string()).c_str() );
  fs::path file_path(file);
  RCLCPP_INFO(this->get_logger(),("file path"+file_path.string()).c_str());
  fs::path full_path = dir_path / file_path;
  RCLCPP_INFO(this->get_logger(),"getDefaultFullFilePath 5.... ");
  RCLCPP_INFO(this->get_logger(), ("File path " + full_path.string()).c_str());
  return full_path.string();
  */

  return "/home/nao/rolling_ws/src/hni/hni_cpp/moves/fear.txt";
  // return "hello.txt";  //WORKING
}

void JointsPlayActionClient::goalResponseCallback(
  const GoalHandleJointsPlay::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void JointsPlayActionClient::feedbackCallback(
  GoalHandleJointsPlay::SharedPtr, const std::shared_ptr<const JointsPlay::Feedback> feedback)
{
  // TODO
}

void JointsPlayActionClient::resultCallback(const GoalHandleJointsPlay::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  if (result.result->success)
    RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");

  rclcpp::shutdown();
}

}  // namespace hni_joints_play_action_client

RCLCPP_COMPONENTS_REGISTER_NODE(hni_joints_play_action_client::JointsPlayActionClient)