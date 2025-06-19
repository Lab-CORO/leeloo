#include "rclcpp/rclcpp.hpp"
// #include "dsr_msgs2/srv/read_data_rt.hpp"
// #include "dsr_msgs2/msg/torque_rt_stream.hpp"
// #include "dsr_msgs2/msg/servl_rt_stream.hpp"
#include "dsr_msgs2/msg/speedj_rt_stream.hpp"
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <pthread.h>
#include <string>
// #include "dsr_realtime_control/rusage_utils.hpp"
// #include "dsr_realtime_control/sched_utils.hpp"
// #include "dsr_realtime_control/command_line_options.hpp"
// #include "dsr_realtime_control/burn_cpu_cycles.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

// #include "../../dsr_common2/include/DRFLEx.h"



class SpeedjRtNode : public rclcpp::Node
{
public:
    explicit SpeedjRtNode();
    virtual ~SpeedjRtNode();

    void speed_rt_stream_timer();

private:  
    rclcpp::Publisher<dsr_msgs2::msg::SpeedjRtStream>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


class ExecuteTrajectoryNode : public rclcpp::Node
{
public:
    explicit ExecuteTrajectoryNode();
    virtual ~ExecuteTrajectoryNode();

    void sub_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void speed_rt_stream_timer();
private:  
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Publisher<dsr_msgs2::msg::SpeedjRtStream>::SharedPtr dsr_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    trajectory_msgs::msg::JointTrajectory trajectory;
    dsr_msgs2::msg::SpeedjRtStream msg_speed;
    std_msgs::msg::Float32 progression; 
    std::mutex mtx;
};