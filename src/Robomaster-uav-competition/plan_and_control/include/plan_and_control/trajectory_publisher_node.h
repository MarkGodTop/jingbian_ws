#ifndef TRAJECTORY_PUBLISHER_NODE_H
#define TRAJECTORY_PUBLISHER_NODE_H

//ros and related msg
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <uav_msgs/DesiredStates.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
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
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber depth_image_sub_;
    cv_bridge::CvImageConstPtr depth_ptr_;
    ros::Publisher desiredStates_pub_, traj_vis_pub_, cmd_vis_pub_;
    ros::Publisher desiredPose_pub_, currentPose_pub_;
    ros::Subscriber odom_sub_, yolo_sub_, gps_sub_;
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
    double startYaw_, finalYaw_, gps_flag_data;
    int resX = 1280;
    int resY = 800;
    int waypoint_flag = 0;
    std::string json_cpp;
    std::string waypoint_flag_data = "Point_1_1";
    std::string cameraName = "D455_RGB_01";
    std::string vehicle = "UnmannedAirplane_1";
    shared_ptr<TrajectoryGeneratorWaypoints> trajPlanWaypoints_ = make_shared<TrajectoryGeneratorWaypoints>();
    bool gps_flag = true;
    std::map<int, std::vector<std::string>> taskMap;
    double distance1 = 0;
    struct MixedValues {
        double x , y, z;
        int class_val;
        double d = 0;
    };
    MixedValues values;
    int cc = 0;
public:

    TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryPublisherNode();
    inline Eigen::Vector3d transformPixel2World(const Eigen::Vector3d &pixel_and_depth) {
        

        Eigen::Matrix3d K;
        K << 640.0, 0.0, 640.0, 0.0, 640.0, 360.0, 0.0, 0.0, 1.0;
        Eigen::Vector3d point_c;
        point_c << (pixel_and_depth(0) - K(0, 2)) * pixel_and_depth(2) / K(0, 0),
                (pixel_and_depth(1) - K(1, 2)) * pixel_and_depth(2) / K(1, 1),
               pixel_and_depth(2);


        Eigen::Matrix3d R_c_b;
        R_c_b << 0, 0, 1, 1, 0, 0, 0, 1, 0;
        Eigen::Vector3d t_c_b (0.5, 0, 0);
        Eigen::Vector3d point_b = R_c_b * point_c + t_c_b;
        Eigen::Matrix3d R_b_w = odom_orient_.normalized().toRotationMatrix();
        Eigen::Vector3d t_b_w = odom_pos_;
        Eigen::Vector3d point_w = R_b_w * point_b + t_b_w;
        cout << "point_w :" << point_w << endl;
        return point_w;
    }
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &waypoints);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();
    void boundingBoxes(const yolov8_ros_msgs::BoundingBoxesConstPtr &msg);
    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif