#include <cstring>
#include <functional>
#include <iostream>  // std::cout
#include <memory>
#include <string>  // std::string, std::stof
#include <thread>
#include <unordered_map>
// #include <vector>

// #include <chrono>

#include "hni_interfaces/action/chat_play.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "hni_cpp/chat_action_client.hpp"

namespace hni_chat_action_client
{

ChatActionClient::ChatActionClient(const rclcpp::NodeOptions& options)
  : rclcpp::Node("chat_action_client_node", options)
{
  using namespace std::placeholders;
  this->client_ptr_ = rclcpp_action::create_client<hni_interfaces::action::ChatPlay>(this, "chat_play");

  this->timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ChatActionClient::sendAsyncGoal, this));

  RCLCPP_INFO(this->get_logger(), "ChatActionClient initialized");
}

ChatActionClient::~ChatActionClient()
{
}

void ChatActionClient::sendAsyncGoal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server())
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = hni_interfaces::action::ChatPlay::Goal();

  auto send_goal_options = rclcpp_action::Client<hni_interfaces::action::ChatPlay>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(&ChatActionClient::goalResponseCallback, this, _1);

  // send_goal_options.feedback_callback =
  //   std::bind(&ChatActionClient::feedbackCallback, this, _1, _2);

  send_goal_options.result_callback = std::bind(&ChatActionClient::resultCallback, this, _1);

  RCLCPP_INFO(this->get_logger(), "Sending goal: ");

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void ChatActionClient::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void ChatActionClient::feedbackCallback(
    rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::SharedPtr,
    const std::shared_ptr<const hni_interfaces::action::ChatPlay::Feedback> feedback)
{
  // TODO
}

void ChatActionClient::resultCallback(
    const rclcpp_action::ClientGoalHandle<hni_interfaces::action::ChatPlay>::WrappedResult& result)
{
  switch (result.code)
  {
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

  // if (result.result->success)
  RCLCPP_INFO(this->get_logger(), "Joints posisitions regulary played.");

  // rclcpp::shutdown();
}

}  // namespace hni_chat_action_client

RCLCPP_COMPONENTS_REGISTER_NODE(hni_chat_action_client::ChatActionClient)