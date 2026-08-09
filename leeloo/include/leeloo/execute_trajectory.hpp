#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/msg/speedj_rt_stream.hpp"
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <pthread.h>
#include <string>

#include <algorithm>
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
    void set_streaming_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr state_publisher_;
    rclcpp::Publisher<dsr_msgs2::msg::SpeedjRtStream>::SharedPtr dsr_publisher_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr streaming_enabled_service_;

    trajectory_msgs::msg::JointTrajectory trajectory;
    dsr_msgs2::msg::SpeedjRtStream msg_speed;
    std_msgs::msg::Float32 progression;
    std::mutex mtx;
    int iteration = 0;
    int trajectory_size = 0;

    // dt contract with curobo_ros (single dt authority — see
    // resolve_interpolation_dt / M7 in the pre-publication audit): a
    // trajectory whose sampled dt does not match command_period_s_ carries
    // velocities computed for a different interval than the one this node
    // will actually apply them over. Since this node pops one point per
    // fixed wall tick and never reads time_from_start, that mismatch is not
    // cosmetic — it is a real position/velocity inconsistency on hardware.
    // sub_trajectory() derives incoming dt and REJECTS the message (does not
    // replace `trajectory`) rather than execute it, on mismatch.
    double command_period_s_ = 0.03;   // timer period; also the idle-path speedj time
    double dt_tolerance_s_ = 0.002;

    // Acceleration streaming (experiment 2026-08-07 — see the constructor's
    // note). When true, publish the planner's own per-joint acceleration in
    // SpeedjRtStream.acc and leave `time` at 0, instead of leaving acc at 0
    // and letting the DRCF derive the acceleration itself from `time`.
    // Read live from the parameter server on every tick, so the two modes can
    // be compared on hardware with `ros2 param set` — no rebuild, no relaunch.
    bool send_acceleration_ = false;
    // Floor/ceiling applied to the published acceleration (deg/s^2). The floor
    // matters: with acc=0 AND time=0 the controller has neither a ramp rate nor
    // a duration, which is exactly the instantaneous velocity step we are
    // trying to avoid — and the first point of every MPC batch does carry
    // acceleration 0. The ceiling mirrors set_accj_rt() in dsr_hw_interface2.cpp
    // so we never request more than the controller accepts (the planner stays
    // well under it: measured peak 28.7 deg/s^2).
    double min_acceleration_dps2_ = 5.0;
    double max_acceleration_dps2_ = 70.0;

    // Toggled by ~/enable_streaming (std_srvs/SetBool). When false, this node
    // stops publishing to speedj_rt_stream entirely (no zero-velocity idle
    // publish either), so dsr_controller2 is free to be driven directly via
    // the Doosan API (movej/movel/drl services on dsr01) without the two
    // control paths fighting over the same streaming interface. Defaults to
    // true to preserve today's behaviour.
    std::atomic<bool> streaming_enabled_{true};
};