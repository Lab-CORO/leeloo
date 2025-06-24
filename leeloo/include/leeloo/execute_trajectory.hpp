#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/msg/speedj_rt_stream.hpp"
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <pthread.h>
#include <string>

#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <cmath>



class ExecuteTrajectoryNode : public rclcpp::Node
{
public:
    explicit ExecuteTrajectoryNode();
    virtual ~ExecuteTrajectoryNode();

    void sub_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void speed_rt_stream_timer();
private:  
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_publisher_;
    rclcpp::Publisher<dsr_msgs2::msg::SpeedjRtStream>::SharedPtr dsr_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    trajectory_msgs::msg::JointTrajectory trajectory;
    dsr_msgs2::msg::SpeedjRtStream msg_speed;
    std_msgs::msg::Float32 progression; 
    std::mutex mtx;
    int iteration = 0;
    int trajectory_size = 0;
};