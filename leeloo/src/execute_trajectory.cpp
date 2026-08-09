
#include "leeloo/execute_trajectory.hpp"


ExecuteTrajectoryNode::ExecuteTrajectoryNode() : Node("ExecuteTrajectory")
{
    // dt contract with curobo_ros (M7 / pre-publication audit): curobo_ros is
    // the single authority on trajectory dt (interpolation_dt), propagated
    // into every outgoing JointTrajectory's time_from_start. This node's own
    // pacing is independent of that field (it pops one point per fixed wall
    // tick), so the two must be kept in sync via parameters instead —
    // defaults below preserve today's hardware behaviour byte-for-byte.
    command_period_s_ = this->declare_parameter("command_period", command_period_s_);
    dt_tolerance_s_ = this->declare_parameter("dt_tolerance", dt_tolerance_s_);

    // Acceleration streaming (experiment 2026-08-07). dsr_controller2 forwards
    // SpeedjRtStream.acc straight into DRFL's speedj_rt(vel, acc, time), but
    // this node never populated `acc` — every command so far went out with
    // acc = {0,...}, so the controller derived the applied acceleration from
    // `time` and the gap between commanded and measured velocity. That gap
    // spikes at MPC batch boundaries: the planner republishes a fresh 16-point
    // batch every ~0.24s while this node consumes one point per 0.03s tick, so
    // point ~7 of a batch is followed immediately by point 0 of the next.
    // Measured on /leeloo/execute_trajectory: 45 deg/s^2 within a batch (the
    // curobo-side clamp saturating exactly at its setting) but 71-127 deg/s^2
    // across the boundary — against the 70 deg/s^2 set_accj_rt() limit, which
    // matches the observed [SpeedJ] "Acceleration is over max value" alarms
    // (72-83 deg/s^2). The planned acceleration itself never exceeds
    // 28.7 deg/s^2, so publishing it and letting the DRCF's own interpolator
    // do the ramping targets the discontinuity without clamping the planner's
    // velocity output (which would fight the MPC rather than help it).
    // `time` keeps its previous value in both modes — see the "NEVER 0" note
    // below; acc is supplied in addition to the duration, not instead of it.
    send_acceleration_ = this->declare_parameter("send_acceleration", send_acceleration_);
    min_acceleration_dps2_ = this->declare_parameter("min_acceleration_dps2", min_acceleration_dps2_);
    max_acceleration_dps2_ = this->declare_parameter("max_acceleration_dps2", max_acceleration_dps2_);

    // {trajectory_msgs/msg/JointTrajectory
    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/leeloo/execute_trajectory", 10, std::bind(&ExecuteTrajectoryNode::sub_trajectory, this, std::placeholders::_1));
    state_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/leeloo/trajectory_state",10); // give a percentage of the advencement
    // Le driver Doosan (dsr_controller2) souscrit à <node_name>/speedj_rt_stream,
    // soit /dsr01/dsr_controller2/speedj_rt_stream (svc_prefix_ = get_name()+"/").
    // Publier sur /dsr01/speedj_rt_stream ne touche personne (0 subscriber).
    dsr_publisher_ = this->create_publisher<dsr_msgs2::msg::SpeedjRtStream>("/dsr01/dsr_controller2/speedj_rt_stream",10);
    // Toggle: leeloo/execute_trajectory pipeline (true, default) vs direct
    // Doosan API control (false — this node goes silent on speedj_rt_stream).
    streaming_enabled_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable_streaming",
        std::bind(&ExecuteTrajectoryNode::set_streaming_enabled, this,
                   std::placeholders::_1, std::placeholders::_2));
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(command_period_s_));
    timer_ = this->create_wall_timer(period, std::bind(&ExecuteTrajectoryNode::speed_rt_stream_timer, this));
}

ExecuteTrajectoryNode::~ExecuteTrajectoryNode()
{
}

void ExecuteTrajectoryNode::speed_rt_stream_timer(){
    if (!this->streaming_enabled_.load()) {
        return;  // Direct Doosan API control in effect — stay silent on speedj_rt_stream.
    }
    if (this->trajectory.points.empty()) {
        this->iteration = 0;
        // if (point.velocities.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            this->msg_speed.vel[i] = 0.0;
        }
        // } else {
            // RCLCPP_WARN(this->get_logger(), "Received fewer than 6 velocity values.");
        // }
        // Idle/stop path stays time-based on purpose: it is the known-working
        // stop command, and acc is meaningless with a zero velocity target.
        // msg_speed is a member, so acc MUST be cleared explicitly here or the
        // last moving point's acceleration would leak into every stop command.
        for (size_t i = 0; i < 6; ++i) {
            this->msg_speed.acc[i] = 0.0;
        }
        this->msg_speed.time = command_period_s_;


    // send traj to dsr
    dsr_publisher_->publish(this->msg_speed);
        return;  // Nothing to do
    }
    if (this->iteration == 0){
        this->trajectory_size = std::size(this->trajectory.points);
    }

        this->mtx.lock();
        trajectory_msgs::msg::JointTrajectoryPoint point = this->trajectory.points.front();
        this->trajectory.points.erase(this->trajectory.points.begin());
        this->mtx.unlock();

        if (point.velocities.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            this->msg_speed.vel[i] = point.velocities[i] * 180 / 3.1415;
        }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received fewer than 6 velocity values.");
        }

        // Read live so the two modes can be A/B'd on hardware via `ros2 param set`.
        const bool send_acc = this->get_parameter("send_acceleration").as_bool();
        if (send_acc && point.accelerations.size() >= 6) {
            const double lo = this->get_parameter("min_acceleration_dps2").as_double();
            const double hi = this->get_parameter("max_acceleration_dps2").as_double();
            for (size_t i = 0; i < 6; ++i) {
                // Magnitude only: the direction of travel is already carried by
                // the velocity target, and set_accj_rt() takes positive limits —
                // a signed acceleration here has no defined meaning for speedj_rt.
                const double a = std::abs(point.accelerations[i]) * 180 / 3.1415;
                this->msg_speed.acc[i] = std::min(std::max(a, lo), hi);
            }
            // NEVER 0. Sending time=0 to signal "acc is authoritative, ignore
            // the duration" was tried on hardware (2026-08-07) and is fatal:
            // the DRCF divides by it, producing `desired position = -nan` and
            // a level-3 alarm 1908 (joint limit check on NaN) within ~0.5s of
            // the first command. `time` stays the command period; `acc` is
            // supplied alongside it, not instead of it.
            this->msg_speed.time = command_period_s_;
        } else {
            for (size_t i = 0; i < 6; ++i) {
                this->msg_speed.acc[i] = 0.0;
            }
            this->msg_speed.time = command_period_s_;
        }


    // send traj to dsr
    dsr_publisher_->publish(this->msg_speed);

    // send feedback
    this->iteration += 1;
    std_msgs::msg::Float32 state_msg;
    state_msg.data = static_cast<float>(this->iteration) / static_cast<float>(this->trajectory_size);
    state_publisher_->publish(state_msg);


}

void ExecuteTrajectoryNode::set_streaming_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    this->streaming_enabled_.store(request->data);
    if (!request->data) {
        // Drop whatever was queued so a stale trajectory doesn't resume
        // mid-flight the moment streaming is re-enabled later.
        this->mtx.lock();
        this->trajectory.points.clear();
        this->mtx.unlock();
    }
    RCLCPP_INFO(this->get_logger(), "speedj_rt_stream publishing %s (%s control)",
                request->data ? "ENABLED" : "DISABLED",
                request->data ? "leeloo/execute_trajectory" : "direct Doosan API");
    response->success = true;
    response->message = request->data
        ? "execute_trajectory now streaming to speedj_rt_stream"
        : "execute_trajectory stopped streaming; direct Doosan API control expected";
}

void ExecuteTrajectoryNode::sub_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
    // dt barrier (M7): an empty message (0 points) is the stop signal this
    // node itself relies on (see the empty-trajectory idle branch above) —
    // it carries no dt to check and MUST always pass through, otherwise a
    // faulty barrier could block the robot's own stop command. A single-point
    // message has no interval to check either.
    if (msg->points.size() >= 2) {
        const auto &t0 = msg->points[0].time_from_start;
        const auto &t1 = msg->points[1].time_from_start;
        double incoming_dt =
            (t1.sec - t0.sec) + (t1.nanosec - t0.nanosec) * 1e-9;
        if (std::abs(incoming_dt - command_period_s_) > dt_tolerance_s_) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Rejecting trajectory: incoming dt=%.4fs does not match "
                "command_period=%.4fs (tolerance=%.4fs, %.1f%% off). "
                "curobo_ros's interpolation_dt param and this node's "
                "command_period param must match. Trajectory NOT applied — "
                "continuing to execute whatever was already queued.",
                incoming_dt, command_period_s_, dt_tolerance_s_,
                100.0 * std::abs(incoming_dt - command_period_s_) / command_period_s_);
            return;  // reject: do not replace the in-flight trajectory
        }
    }

    // call mutex
    this->mtx.lock();
    this->trajectory = *msg;
    this->mtx.unlock();
}


int main(int argc, char * argv[])
{

    // create two executor one for a timer at 100Hz and the other for a subscriber to get the traj or stop the robot.
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ExecuteTrajectoryNode>());
    rclcpp::shutdown();

    return 0;
}

