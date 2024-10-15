#include "plan_and_control/trajectory_publisher_node.h"

msr::airlib::MultirotorRpcLibClient client("192.168.1.51");
TrajectoryPublisherNode::TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
:nh_(nh), nh_private_(nh_private)  {
    
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/drone_1/odom_local_ned",1, &TrajectoryPublisherNode::odomCallback,this);
    
}


TrajectoryPublisherNode::~TrajectoryPublisherNode() {}





void TrajectoryPublisherNode::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_ = msg;
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;
    if(odom_pos_(2) >= 0.2 && stamp1 != odom_->header.stamp){
        stamp1 = odom_->header.stamp;
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
        client.simUpdateLocalPositionData(vehicle, odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z, ms, 7);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");
 
    TrajectoryPublisherNode trajectoryPublisherNode(nh, nh_p);

    ros::spin();
    return 0;
}