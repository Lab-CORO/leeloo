
#include "leeloo/execute_trajectory.hpp"


ExecuteTrajectoryNode::ExecuteTrajectoryNode() : Node("ExecuteTrajectory")
{
    // {trajectory_msgs/msg/JointTrajectory
    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/leeloo/execute_trajectory", 10, std::bind(&ExecuteTrajectoryNode::sub_trajectory, this, std::placeholders::_1));   
    state_publisher_  = this->create_publisher<std_msgs::msg::Float32>("/leeloo/trajectory_state",10); // give a percentage of the advencement
    dsr_publisher_ = this->create_publisher<dsr_msgs2::msg::SpeedjRtStream>("/dsr01/speedj_rt_stream",10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ExecuteTrajectoryNode::speed_rt_stream_timer, this));
}

ExecuteTrajectoryNode::~ExecuteTrajectoryNode()
{
}

void ExecuteTrajectoryNode::speed_rt_stream_timer(){
    if (this->trajectory.points.empty()) {
        this->iteration = 0;
        // if (point.velocities.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            this->msg_speed.vel[i] = 0.0;
        }
        // } else {
            // RCLCPP_WARN(this->get_logger(), "Received fewer than 6 velocity values.");
        // }
        this->msg_speed.time = 0.02; 

        
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
        this->msg_speed.time = 0.02; 

        
    // send traj to dsr 
    dsr_publisher_->publish(this->msg_speed);

    // send feedback 
    this->iteration += 1;
    std_msgs::msg::Float32 state_msg;
    state_msg.data = static_cast<float>(this->iteration) / static_cast<float>(this->trajectory_size);
    state_publisher_->publish(state_msg);

   
}

void ExecuteTrajectoryNode::sub_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
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

