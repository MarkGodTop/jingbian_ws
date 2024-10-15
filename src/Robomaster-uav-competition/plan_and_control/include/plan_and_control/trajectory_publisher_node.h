#ifndef TRAJECTORY_PUBLISHER_NODE_H
#define TRAJECTORY_PUBLISHER_NODE_H

//ros and related msg
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <uav_msgs/DesiredStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <yolov8_ros_msgs/BoundingBox.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
//other utils
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include <iostream>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <regex>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "trajectory_generator/mini_jerk_traj.h"

using namespace std;
using namespace Json;
using namespace cv;
class TrajectoryPublisherNode
{
private:
    //ros related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher desiredStates_pub_, traj_vis_pub_, cmd_vis_pub_;
    ros::Publisher desiredPose_pub_, currentPose_pub_;
    ros::Subscriber odom_sub_, yolo_sub_;
    ros::ServiceClient takeoff_client_;
    uav_msgs::DesiredStates cmd_;
    nav_msgs::OdometryConstPtr odom_;
    geometry_msgs::PoseStamped desiredPose_, currentPose_;
    ros::Time stamp1;
    ros::Time stamp3;
    //parameters for planner
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desiredPos_;
    Eigen::Vector3d dir_;
    Eigen::VectorXd times_;
    Eigen::MatrixXd coeffMatrix_;
    Eigen::MatrixXd waypoints_;
    int waypoint_num_;
    double max_vel_, max_acc_, max_jerk_;
    ros::Time startTime_ ;
    ros::Time finalTime_ ;
    int segment_num_;
    double trajDuration_;
    yolov8_ros_msgs::BoundingBoxesConstPtr yolo_;
    double startYaw_, finalYaw_;
    int resX = 800;
    int resY = 600;
    std::string cameraName = "D455_RGB_01";
    std::string vehicle = "UnmannedAirplane_1";
    shared_ptr<TrajectoryGeneratorWaypoints> trajPlanWaypoints_ = make_shared<TrajectoryGeneratorWaypoints>();

public:

    TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryPublisherNode();
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &waypoints);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();
    void boundingBoxes(const yolov8_ros_msgs::BoundingBoxesConstPtr &msg);
    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
};

#endif