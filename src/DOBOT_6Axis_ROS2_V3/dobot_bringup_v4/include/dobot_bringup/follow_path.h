    
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/create_server.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <dobot_bringup/command.h>
#include <dobot_bringup/parseTool.h>
class FollowPath{

    rclcpp::Node &node;
        
    public:
    std::shared_ptr<CRCommanderRos2> commander_;
    FollowPath( rclcpp::Node &node):node(node){

    }
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>;
    void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
    bool ServoJ(const std::shared_ptr<dobot_msgs_v4::srv::ServoJ::Request> request, const std::shared_ptr<dobot_msgs_v4::srv::ServoJ::Response> response);
    rclcpp::Logger get_logger(){
        return node.get_logger();
    }
    rclcpp::Clock::SharedPtr get_clock(){
        return node.get_clock();
    }
};
