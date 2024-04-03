#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>
#include <string>     // std::string, std::stof


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nao_lola_sensor_msgs/msg/joint_indexes.hpp"
#include "nao_lola_sensor_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_positions.hpp"
#include "nao_lola_command_msgs/msg/joint_indexes.hpp"
#include "nao_lola_command_msgs/msg/joint_stiffnesses.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointsRecorder : public rclcpp::Node {

 public:

  JointsRecorder()
    : Node("puppeteer") {
    jpos_sub_ = this->create_subscription<nao_lola_sensor_msgs::msg::JointPositions>(
                  "sensors/joint_positions", 10, std::bind(&JointsRecorder::jpos_callback, this, _1));
    record_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                        "record_cmd_topic", 10, std::bind(&JointsRecorder::record_cmd_callback, this, _1));
    play_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                      "play_cmd_topic", 10, std::bind(&JointsRecorder::play_cmd_callback, this, _1));

    jpos_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointPositions>(
                  "effectors/joint_positions", 10);
    jstiff_pub_ = this->create_publisher<nao_lola_command_msgs::msg::JointStiffnesses>(
                    "effectors/joint_stiffnesses", 10);

    //jstiff_cmd_.indexes.push_back(nao_lola_sensor_msgs::msg::JointIndexes::LHIPYAWPITCH);
    //jstiff_cmd_.stiffnesses.push_back(1.0);
    //jstiff_pub_->publish(jstiff_cmd_);

    RCLCPP_INFO_STREAM(this->get_logger(), "JointsRecorder node initialized");
  }

 private:

  /*enum movingJoints : unsigned int {
    LSHOULDERPITCH = nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERPITCH;
    LSHOULDERROLL = nao_lola_sensor_msgs::msg::JointIndexes::LSHOULDERROLL;
    LELBOWYAW = nao_lola_sensor_msgs::msg::JointIndexes::LELBOWYAW;
    LELBOWROLL = nao_lola_sensor_msgs::msg::JointIndexes::LELBOWROLL;
    LWRISTYAW = nao_lola_sensor_msgs::msg::JointIndexes::LWRISTYAW;
    RSHOULDERPITCH = nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERPITCH;
    RSHOULDERROLL = nao_lola_sensor_msgs::msg::JointIndexes::RSHOULDERROLL;
    RELBOWYAW = nao_lola_sensor_msgs::msg::JointIndexes::RELBOWYAW;
    RELBOWROLL = nao_lola_sensor_msgs::msg::JointIndexes::RELBOWROLL;
    RWRISTYAW = nao_lola_sensor_msgs::msg::JointIndexes::RWRISTYAW;
  }*/

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr record_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr play_cmd_sub_;
  rclcpp::Subscription<nao_lola_sensor_msgs::msg::JointPositions>::SharedPtr jpos_sub_;

  rclcpp::Publisher<nao_lola_command_msgs::msg::JointPositions>::SharedPtr jpos_pub_;
  rclcpp::Publisher<nao_lola_command_msgs::msg::JointStiffnesses>::SharedPtr jstiff_pub_;

  bool recording_ = false;
  nao_lola_sensor_msgs::msg::JointIndexes joint_indexes_msg_;
  uint8_t rec_joint_indexes_ [10] =  {  joint_indexes_msg_.LSHOULDERPITCH,
                                        joint_indexes_msg_.LSHOULDERROLL,
                                        joint_indexes_msg_.LELBOWYAW,
                                        joint_indexes_msg_.LELBOWROLL,
                                        joint_indexes_msg_.LWRISTYAW,
                                        joint_indexes_msg_.RSHOULDERPITCH,
                                        joint_indexes_msg_.RSHOULDERROLL,
                                        joint_indexes_msg_.RELBOWYAW,
                                        joint_indexes_msg_.RELBOWROLL,
                                        joint_indexes_msg_.RWRISTYAW
                                     };

  uint8_t num_rec_joints_ = sizeof(rec_joint_indexes_) / sizeof(rec_joint_indexes_[0]);
  std::vector<float> record_;
  std::fstream file_;
  nao_lola_command_msgs::msg::JointPositions jpos_cmd_;
  nao_lola_command_msgs::msg::JointStiffnesses jstiff_cmd_;



  void jpos_callback(const nao_lola_sensor_msgs::msg::JointPositions & joints) {

    if (recording_) {

      for (auto i : rec_joint_indexes_) {
        record_.emplace_back(joints.positions[i]);
      }
      RCLCPP_DEBUG_STREAM(this->get_logger(), "New joint positions saved");

    }

  }

  void record_cmd_callback(const std_msgs::msg::Bool & msg)  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.data << "' from record_cmd_sub");
    bool cmd = msg.data;

    if (!recording_ && cmd) {

      for (int i : rec_joint_indexes_) {
        jstiff_cmd_.indexes.push_back(i);
        jstiff_cmd_.stiffnesses.push_back(0.0);
      }
      jstiff_pub_->publish(jstiff_cmd_);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing 0.0 on effectors/joint_stiffnesses");
      jstiff_cmd_.indexes.clear();
      jstiff_cmd_.stiffnesses.clear();

      rclcpp::sleep_for(1s);

      recording_ = true;
      RCLCPP_INFO_STREAM(this->get_logger(), "Start recording");

    } else if (recording_ && !cmd) {
      // save the data
      recording_ = false;
      RCLCPP_INFO_STREAM(this->get_logger(), "Stop recording");

      file_.open("vector_file_.txt", std::ios_base::out);
      if (file_.is_open()) {
        for (uint i = 0; i < record_.size(); i++) { //-1 to avoid a final \n line
          if (i < record_.size() - 1)
            file_ << record_[i] << '\n';
          else
            file_ << record_[i];
        }

        file_.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Joint position saved to file");


      } else
        std::cout << "Unable to open file";
    }
  }

  void play_cmd_callback(const std_msgs::msg::Bool & msg)  {

    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.data << "' from play_cmd_sub");
    //rclcpp::sleep_for(1s);
    bool cmd = msg.data;
    //RCLCPP_WARN_STREAM(this->get_logger(), "I heard: '" << cmd);
    //rclcpp::sleep_for(1s);


    if (!recording_ && cmd) {

      for (auto i : rec_joint_indexes_) {
        jstiff_cmd_.indexes.push_back(i);
        jstiff_cmd_.stiffnesses.push_back(1.0);
      }
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing 1.0 on effectors/joint_stiffnesses");
      //rclcpp::sleep_for(1s);

      jstiff_pub_->publish(jstiff_cmd_);
      jstiff_cmd_.indexes.clear();
      jstiff_cmd_.stiffnesses.clear();
      

      //file_.open("vector_file_.txt", std::ios_base::in);
      file_.open("vector_file_.txt", std::fstream::in);
      if (file_.is_open()) {
        record_.clear();
        float joint_value;
        std::string line;
        while (!file_.eof()) {
          std::getline(file_, line, '\n');
          //std::cout << line;  
          joint_value = std::stof(line);
          record_.emplace_back( joint_value );
        }
        file_.close();
      } else {
        return;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing on effectors/joint_positions 82 Hz");
      rclcpp::Rate loop_rate(82);
      for (uint64_t i = 0; i < record_.size(); i += num_rec_joints_) {
        for (uint8_t j = 0; j < num_rec_joints_; j++ ) {
          jpos_cmd_.indexes.push_back(rec_joint_indexes_[j]);
          jpos_cmd_.positions.push_back(record_[i + j]);
        }
        jpos_pub_->publish(jpos_cmd_);
        jpos_cmd_.indexes.clear();
        jpos_cmd_.positions.clear();

        loop_rate.sleep();

        //rclcpp::sleep_for(12ms); // 80hz nao_lola update rate
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "Play finished.");

    }
  }

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointsRecorder>());
  rclcpp::shutdown();
  return 0;
}