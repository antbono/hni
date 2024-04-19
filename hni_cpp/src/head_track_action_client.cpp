#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/filesystem.hpp"
#include "hni_interfaces/action/video_tracker.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace fs = boost::filesystem;

namespace hni_head_track_cpp
{

class HeadTrackActionClient : public rclcpp::Node
{
public:
  using ObjTrack = hni_interfaces::action::VideoTracker;
  using GoalHandleObjTrack = rclcpp_action::ClientGoalHandle<ObjTrack>;

  explicit HeadTrackActionClient(const rclcpp::NodeOptions & options)
  : Node("head_track_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<ObjTrack>(this, "head_track");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&HeadTrackActionClient::send_goal, this));

    // this->declare_parameter<std::string>("file", getDefaultFullFilePath());

    RCLCPP_INFO(this->get_logger(), "HeadTrackActionClient initialized");
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = ObjTrack::Goal();

    goal_msg.camera_id = " ";

    auto send_goal_options = rclcpp_action::Client<ObjTrack>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&HeadTrackActionClient::goal_response_callback, this, _1);

    // send_goal_options.feedback_callback =
    //   std::bind(&HeadTrackActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&HeadTrackActionClient::result_callback, this, _1);

    RCLCPP_INFO(this->get_logger(), "Sending goal: ");

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ObjTrack>::SharedPtr client_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string getDefaultFullFilePath()
  {
    /*
    RCLCPP_INFO(this->get_logger(),"getDefaultFullFilePath ");
    std::string file = "moves/hello.txt";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(
                                            "hni_moves");
    // e.g. /home/nao/rolling_ws/install/hni_moves/share/hni_moves

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

    return "/home/nao/rolling_ws/src/hni/hni_moves/moves/fear.txt";
    // return "hello.txt";  //WORKING
  }

  void goal_response_callback(const GoalHandleObjTrack::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleObjTrack::SharedPtr, const std::shared_ptr<const ObjTrack::Feedback> feedback)
  {
    // TODO
  }

  void result_callback(const GoalHandleObjTrack::WrappedResult & result)
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

};  // class HeadTrackActionClient

}  // namespace hni_head_track_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(hni_head_track_cpp::HeadTrackActionClient)