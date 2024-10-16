#ifndef TRAJECTORY_REPLAN_H
#define TRAJECTORY_REPLAN_H

#include <ros/ros.h>
//for trajectory planning
#include <uav_msgs/DesiredStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_msgs/Takeoff.h>
#include <iostream>
//for cv
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yolov8_ros_msgs/BoundingBox.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include "trajectory_generator/mini_jerk_traj.h"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
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
// #include <yaml-cpp/yaml.h>

using namespace Json;
using namespace std;
using namespace cv;

class ThreadPool {
public:
    ThreadPool(size_t threads) : stop(false) {
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty()) return;
                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template<class F, class... Args>
    void enqueue(F&& f, Args&&... args) {
        std::function<void()> task(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace(std::move(task));
        }
        condition.notify_one();
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers)
            worker.join();
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    std::atomic<bool> stop;
};

class TrajectoryReplanNode {

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    //for trajectory planning
    ros::Publisher desiredStates_pub_, traj_vis_pub_, cmd_vis_pub_;
    ros::Publisher desiredPose_pub_, currentPose_pub_, Pose_pub_;
    ros::Subscriber odom_sub_, yolo_sub_;
    ros::Timer timer_;
    uav_msgs::DesiredStates cmd_;
    yolov8_ros_msgs::BoundingBoxesConstPtr yolo_;
    nav_msgs::OdometryConstPtr odom_;
    geometry_msgs::PoseStamped desiredPose_, currentPose_;
    ros::ServiceClient takeoff_client_;
    //parameters for trajectory planning
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;
    Eigen::Vector3d desired_pos_;
    Eigen::Vector3d dir_;
    Eigen::VectorXd times_;
    Eigen::MatrixXd coeff_matrix_;
    Eigen::MatrixXd waypoints_;
    std::map<int, std::vector<std::string>> taskMap;
    int waypoint_num_;
    double max_vel_, max_acc_, max_jerk_;
    ros::Time start_time_ ;
    ros::Time final_time_ ;
    int segment_num_;
    double traj_duration_;
    double start_yaw_, final_yaw_;
    bool got_circle_flag_;
    int row_idx_ = 1;
    int resX = 800;
    int resY = 600;
    std::string cameraName = "Custom_MV_CAMERA_001_01";
    std::string vehicle;
    std::string json_cpp;
    bool reached_waypoint = false;
    double distance1 = 0;
    std::atomic<bool> continuePublishing = true;
    std::atomic<int> flag1 = 0; 
    std::atomic<int> num = 0;
    ros::Time stamp1;
    ros::Time stamp2;
    ros::Time stamp3;
    int cc = 0;

std::vector<std::shared_ptr<std::thread>> publishThreads; // 用于存储线程对象
std::unordered_map<std::string, int> set_t;
    //for cv
    std::unique_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_image_sub_;
    image_transport::Subscriber depth_image_sub_;
    cv_bridge::CvImageConstPtr color_ptr_, depth_ptr_;
    vector<vector<Point>> pt_;
    struct MixedValues {
        double x , y, z;
        int class_val;
        double d = 0;
    };
    MixedValues values;

    shared_ptr<TrajectoryGeneratorWaypoints> trajPlanWaypoints_ = make_shared<TrajectoryGeneratorWaypoints>();
public:
    TrajectoryReplanNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~TrajectoryReplanNode();
    
    inline Eigen::Vector3d transformPixel2World(const Eigen::Vector3d &pixel_and_depth) {
        
        // cout << "ceicle Center in pixel:" << point_p.transpose() << endl;
        Eigen::Matrix3d K;
        K << 400.0, 0.0, 400.0, 0.0, 400.0, 300.0, 0.0, 0.0, 1.0;
        Eigen::Vector3d point_c;
        point_c << (pixel_and_depth(0) - K(0, 2)) * pixel_and_depth(2) / K(0, 0),
                (pixel_and_depth(1) - K(1, 2)) * pixel_and_depth(2) / K(1, 1),
               pixel_and_depth(2);
        // Eigen::Vector3d point_c = point_p;
        //cout << "ceicle Center in camera:" << point_c.transpose() << endl;

        Eigen::Matrix3d R_c_b;
        R_c_b << 0, 0, 1, 1, 0, 0, 0, 1, 0;
        Eigen::Vector3d t_c_b (0.5, 0, 0);
        Eigen::Vector3d point_b = R_c_b * point_c + t_c_b;
        //cout << "ceicle Center in body:" << point_b.transpose() << endl;
        Eigen::Matrix3d R_b_w = odom_orient_.normalized().toRotationMatrix();
        //Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_b_w = odom_pos_;
        Eigen::Vector3d point_w = R_b_w * point_b + t_b_w;
        //cout << "uav current pos:" << t_b_w.transpose() << endl;
        // Eigen::Quaterniond q = odom_orient_.normalized();
        // Eigen::Isometry3d T_wb(q);
        // T_wb.pretranslate(odom_pos_);
        // Eigen::Vector3d point_w = T_wb * point_b;
        // cout << "ceicle Center in world:" << point_w.transpose() << endl;
        //cout << "distance to center:" << (point_w - odom_pos_).norm() << endl;
        cout << "point_w :" << point_w << endl;
        return point_w;
    }
    void boundingBoxes(const yolov8_ros_msgs::BoundingBoxesConstPtr &msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    Eigen::VectorXd timeAllocation(const Eigen::MatrixXd &waypoints);
    void trajectoryGenerate(const Eigen::MatrixXd &waypoints);
    void desiredStatesPub();
    void shared_yolo();
    void publishTopic_yolo();
    void publishTopic_dingzi(const int& data);
    void publishTopic_dingwei();
    void publishTopic_ceju(const int& data);
    void publishTopic_jihui(const int& data);
    void displayTrajWithColor();
    void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                 const Eigen::Vector4d& color);
    //get depth image and get circle center
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getCircleCenter(const ros::TimerEvent &e);
    void periodicCallback(const ros::TimerEvent &e);
    
};


#endif