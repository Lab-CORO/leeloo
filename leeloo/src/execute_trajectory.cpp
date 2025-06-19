
#include "leeloo/execute_trajectory.hpp"


SpeedjRtNode::SpeedjRtNode() : Node("SpeedjRt")
{
    publisher_  = this->create_publisher<dsr_msgs2::msg::SpeedjRtStream>("/dsr01/speedj_rt_stream",10);
    timer_      = this->create_wall_timer(std::chrono::microseconds(1000),std::bind(&SpeedjRtNode::speed_rt_stream_timer,this));

}

SpeedjRtNode::~SpeedjRtNode()
{
}

void SpeedjRtNode::speed_rt_stream_timer(){
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
}

ExecuteTrajectoryNode::ExecuteTrajectoryNode() : Node("ExecuteTrajectory")
{
    // {trajectory_msgs/msg/JointTrajectory
    subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>("/leeloo/execute_trajectory", 10, std::bind(&ExecuteTrajectoryNode::sub_trajectory, this, std::placeholders::_1));   
    state_publisher_  = this->create_publisher<std_msgs::msg::String>("/leeloo/trajectory_state",10); // give a percentage of the advencement
    dsr_publisher_ = this->create_publisher<dsr_msgs2::msg::SpeedjRtStream>("/dsr01/speedj_rt_stream",10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ExecuteTrajectoryNode::speed_rt_stream_timer, this));

}

ExecuteTrajectoryNode::~ExecuteTrajectoryNode()
{
}

void ExecuteTrajectoryNode::speed_rt_stream_timer(){
    if (this->trajectory.points.empty()) {
        // RCLCPP_INFO(this->get_logger(), "Hello from ROS2");
        // this->msg_speed.vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        return;  // Nothing to do
    }
    this->mtx.lock();
    // trajectory_msgs::msg::JointTrajectoryPoint point  =  this->trajectory.points.pop_front() 
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
        // std::copy(point.velocities.begin(), point.velocities.begin() + 6, this->msg_speed.vel.begin());
        this->msg_speed.time = 0.01;
        
    // send traj to dsr 
    dsr_publisher_->publish(this->msg_speed);
    

    
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
    // auto node1= std::make_shared<ExecuteTrajectoryNode>();
    // rclcpp::executors::SingleThreadedExecutor executor1;
    // executor1.add_node(node1);
    // auto executor1_thread = std::thread([&](){executor1.spin();});

    // auto node2= std::make_shared<SpeedjRtNode>();
    // rclcpp::executors::SingleThreadedExecutor executor2;
    // executor2.add_node(node2);
    // auto executor2_thread = std::thread([&](){executor2.spin();});

    // executor1_thread.join();
    // executor2_thread.join();

    
    // rclcpp::shutdown();
    return 0;
}

