#include "dobot_bringup/follow_path.h"
double toSec(builtin_interfaces::msg::Duration duration){
    return duration.sec + (duration.nanosec/1000000000.0);
}
std::vector<double> sample_traj(const trajectory_msgs::msg::JointTrajectoryPoint& P0,
                                         const trajectory_msgs::msg::JointTrajectoryPoint& P1, const double& time_index)
{
    double a, b, c, d;
    double T = toSec(P1.time_from_start) - toSec(P0.time_from_start);
    double t = time_index;
    std::vector<double> interp_traj;
    for (int i = 0; i < P0.positions.size(); i++) {
        a = P0.positions[i];
        b = P0.velocities[i];
        c = (-3.0 * P0.positions[i] + 3.0 * P1.positions[i] - 2.0 * T * P0.velocities[i] - T * P1.velocities[i]) /
            (T * T);
        d = (2.0 * P0.positions[i] - 2.0 * P1.positions[i] + T * P0.velocities[i] + T * P1.velocities[i]) / (T * T * T);
        interp_traj.push_back((a + b * t + c * t * t + d * t * t * t) * 180.0 / M_PI);
    }
    return interp_traj;
}

bool FollowPath::ServoJ(const std::shared_ptr<dobot_msgs_v4::srv::ServoJ::Request> request, const std::shared_ptr<dobot_msgs_v4::srv::ServoJ::Response> response)
{
    // return true;
    return commander_->callRosService(parseTool::parserServoJRequest2String(request), response->res);
}


void FollowPath::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
auto goal = goal_handle->get_goal();

    static const double SERVOJ_DURATION = 0.032;
    double t = SERVOJ_DURATION ;
    // ros::Rate timer(1.0 / SERVOJ_DURATION);    // servoj发布频率
    double t0 = get_clock()->now().seconds();

    try {
        for (int i = 0; i < goal->trajectory.points.size() - 1; i++) {
            trajectory_msgs::msg::JointTrajectoryPoint interp_traj_begin = goal->trajectory.points[i];
            trajectory_msgs::msg::JointTrajectoryPoint interp_traj_end = goal->trajectory.points[i + 1];
            double real_time;    // 实际间隔时间
            double t1;
            t1 = get_clock()->now().seconds();
            real_time = t1 - t0;
            RCLCPP_INFO(this->get_logger(), "start: %f end: %f", toSec(interp_traj_begin.time_from_start), toSec(interp_traj_end.time_from_start));
            while (real_time < toSec(interp_traj_end.time_from_start) - SERVOJ_DURATION) {
                const auto start{std::chrono::system_clock::now()};
                double time_index = real_time - toSec(interp_traj_begin.time_from_start);
                std::vector<double> tmp = sample_traj(interp_traj_begin, interp_traj_end, time_index);
                char cmd[100];
                RCLCPP_INFO(this->get_logger(), "ServoJ({%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},t=%0.3f,aheadtime=50,gain=500)", tmp[0], tmp[1], tmp[2], tmp[3],
                        tmp[4], tmp[5], t);
                int32_t err_id;
                auto request = std::make_shared<dobot_msgs_v4::srv::ServoJ::Request>();
                request->a = tmp[0];
                request->b = tmp[1];
                request->c = tmp[2];
                request->d = tmp[3];
                request->e = tmp[4];
                request->f = tmp[5];
                request->t = t;
                auto response = std::make_shared<dobot_msgs_v4::srv::ServoJ::Response>();
                
                ServoJ(request, response);
                std::this_thread::sleep_until(start + std::chrono::milliseconds((int)(SERVOJ_DURATION*1000)));
                t1 = get_clock()->now().seconds();
                real_time = t1 - t0;
            }
        }
        std::vector<double> last_traj;
        int point_num = goal->trajectory.points.size();
        for (int i = 0; i < 6; i++) {
            last_traj.push_back(goal->trajectory.points[point_num - 1].positions[i] * 180 / M_PI);
        }
        char cmd[100];
        RCLCPP_INFO(this->get_logger(), "ServoJ({%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f},t=%0.3f,aheadtime=50,gain=500)", last_traj[0], last_traj[1], last_traj[2],
                last_traj[3], last_traj[4], last_traj[5], t);
        int32_t err_id;
        auto response = std::make_shared<dobot_msgs_v4::srv::ServoJ::Response>();
    auto request = std::make_shared<dobot_msgs_v4::srv::ServoJ::Request>();
        request->a = last_traj[0];
        request->b = last_traj[1];
        request->c = last_traj[2];
        request->d = last_traj[3];
        request->e = last_traj[4];
        request->f = last_traj[5];
        request->t = SERVOJ_DURATION;
                ServoJ(request, response);
    } catch (const TcpClientException& err) {
         RCLCPP_ERROR(this->get_logger(), "Goal error");
        return;
    }
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

