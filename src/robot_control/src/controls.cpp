#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <repair_interface/action/repair_action.hpp>
#include "Serial.h"  // Adjust include path as needed
  

using namespace std::placeholders;
using Twist = geometry_msgs::msg::Twist;
using RepairCmd = repair_interface::action::RepairAction;
using RepairGoalHandle = rclcpp_action::ServerGoalHandle<RepairCmd>;

//using RelayControl = ur16_arc_interface::srv::RelayControl;

class CmdToSerial : public rclcpp::Node {
public:
  CmdToSerial() : Node("cmd_to_serial") {
    cb_group = this-> create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    serial_ = std::make_shared<Serial>("/dev/ttyACM0");
  
    vel_sub_ = create_subscription<Twist>("cmd_vel", 10,
      std::bind(&CmdToSerial::VelCallback, this, _1));

    tool_server = rclcpp_action::create_server<RepairCmd> (
      this,
      "tool_server",
      std::bind(&CmdToSerial::goalCb, this, _1, _2),
      std::bind(&CmdToSerial::cancelCb, this, _1),
      std::bind(&CmdToSerial::executeCb, this, _1),
      rcl_action_server_get_default_options(), cb_group );
      
    
    
    RCLCPP_INFO(this->get_logger(), "Node subscribed to cmd_vel and tool service started");
  }

private:
  void VelCallback(const Twist::SharedPtr msg) {
    // Format the command string as: "move <linear.x> <angular.z>"
    std::string cmd = "move " + std::to_string(msg->linear.x) + " " + std::to_string(msg->angular.z);
    
    // Write the command string to the serial port.
    serial_->write(cmd);
    
    RCLCPP_INFO(this->get_logger(), "Sent move command: %s", cmd.c_str());
  }
  rclcpp_action::GoalResponse goalCb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RepairCmd::Goal> goal) {
    (void) uuid;
    RCLCPP_INFO(this -> get_logger(), "Received goal...");
    if (goal->command.empty()){
      RCLCPP_INFO(this-> get_logger(), "Rejecting goal, operation not valid");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal Accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void executeCb(const std::shared_ptr<RepairGoalHandle> goal_handle){
    std::string cmd = goal_handle -> get_goal() ->command;
    auto result = std::make_shared<RepairCmd::Result>();
    RCLCPP_INFO(this->get_logger(), "Executing '%s' operation", cmd.c_str());
    if(cmd == "tool_lock") {
      serial_->write(cmd);
    }
    if(cmd == "tool_unlock") {
      serial_->write(cmd);
    }
    result -> completed = true;
    goal_handle->succeed(result);
  }

  rclcpp_action::CancelResponse cancelCb (const std::shared_ptr<RepairGoalHandle> goal_handle){
    (void) goal_handle;
    RCLCPP_INFO(this->get_logger(), "Cancel Request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  rclcpp_action::Server<RepairCmd>::SharedPtr tool_server;
  rclcpp::CallbackGroup::SharedPtr cb_group;
  std::shared_ptr<Serial> serial_;
  rclcpp::Subscription<Twist>::SharedPtr vel_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdToSerial>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
