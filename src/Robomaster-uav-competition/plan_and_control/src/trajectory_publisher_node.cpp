#include "plan_and_control/trajectory_publisher_node.h"

msr::airlib::MultirotorRpcLibClient client("192.168.1.123");
TrajectoryPublisherNode::TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
:nh_(nh), nh_private_(nh_private)  {
    Json::Value root;
    Json::CharReaderBuilder builder;
    nh_private_.getParam("json_cpp", json_cpp);
    std::ifstream i("/home/ros20/jingbian_ws/269.json", std::ifstream::binary);
    std::string errs;
    if (!Json::parseFromStream(builder, i, &root, &errs)) { 
        std::cerr << "Error opening or parsing JSON file: " << errs << std::endl;
        return;
    }
    i.close();
    const Json::Value path_data = root["TaskData"]["EquipmentData"][0]["PathData"];
    int index = 0;
    max_vel_ = 10;
    max_acc_ = 4;
    for (const auto& point : path_data)
        ++index;
    waypoint_num_ = index;
    waypoints_.resize(waypoint_num_, 3);
    index = 0;
    for (const auto& point : path_data) {
        waypoints_(index,0) = (point["PointX"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointX"].asDouble())/100;
        waypoints_(index,1) = (point["PointY"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointY"].asDouble())/100;
        waypoints_(index,2) = (point["PointZ"].asDouble() - root["TaskData"]["EquipmentData"][0]["PathData"][0]["PointZ"].asDouble())/100;
        int i = 1;
        // 获取 PointID 和 TaskIndicator
        int pointID = point["PointID"].asInt();
        const Json::Value& taskIndicator = point["TaskIndicator"];
        // 遍历 TaskIndicator 中的每个任务
        for (const auto& task : taskIndicator) {
            // 如果是数组，则表示一个任务和其持续时间
            std::string taskName = task["TaskIndicatorName"].asString();
            std::string taskDuration = task["TaskIndicatorEnd"].asString();
            taskMap[pointID].emplace_back(taskName + "(" + taskDuration + ")");
        }
        ++index;
    }
        
        // YAML::Node task_node(YAML::NodeType::Sequence);
        // for (const auto& task : taskIndicator) {
        //     std::string task_name = "task" + std::to_string(i);
        //     std::string persistent_name = "persistent" + std::to_string(i);
        //     if (task.isArray()) {
        //         YAML::Node task_info(YAML::NodeType::Map);
        //         task_info[task_name] = task[0].asString();
        //         task_info[persistent_name] = task[1].asInt();
        //         // task_node.emplace_back(task_info);
        //     } else if (task.isString()) {
        //         YAML::Node task_info(YAML::Node Type::Map);
        //         task_info[task_name] = task.asString();
        //         // task_node.emplace_back(task_info);
        //     }
        // i++;
        // }
        // point_node["TaskIndicator"] = task_node;

        // 将节点添加到YAML节点中
        // std::string key = "waypoint" + std::to_string(index);
        // node[key] = point_node;
    
    for (const auto& taskList : taskMap) {
        std::cout << "PointID: " << taskList.first << std::endl;
        for (const auto& task : taskList.second) {
            std::cout << "Task: " << task << std::endl;
        }
        std::cout << std::endl;
    }
    it_ = std::make_unique<image_transport::ImageTransport>(nh_);
    depth_image_sub_ = it_->subscribe("/airsim_node/UnmannedAirplane_1/D455_Depth_01/DepthPerspective", 10,
                        std::bind(&TrajectoryPublisherNode::depthImageCallback, this,  std::placeholders::_1));
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/UnmannedAirplane_1/odom_local_ned",1, &TrajectoryPublisherNode::odomCallback,this);
    yolo_sub_ = 
        nh_.subscribe<yolov8_ros_msgs::BoundingBoxes>("/yolov8/BoundingBoxes",10, &TrajectoryPublisherNode::boundingBoxes,this);
    // gps_sub_ =
    //     nh_.subscribe<sensor_msgs::NavSatFix>("/airsim_node/UnmannedAirplane_1/gps/GPS_tms_3",1, &TrajectoryPublisherNode::gpsCallback,this);
    
}
void TrajectoryPublisherNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if(yolo_ == nullptr){
        return;
    }
    for (std::size_t i = 0; i < yolo_->bounding_boxes.size(); ++i){
        float x_center_rgb = (yolo_->bounding_boxes[i].xmin + yolo_->bounding_boxes[i].xmax) / 2.0f;
        float y_center_rgb = (yolo_->bounding_boxes[i].ymin + yolo_->bounding_boxes[i].ymax) / 2.0f * 0.9;
        depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        Mat depth = depth_ptr_->image;
        int y = y_center_rgb;
        int x = x_center_rgb;
        float depth_value = depth.ptr<float>(y)[x];
        Eigen::Vector3d pixel_and_depth(x_center_rgb, y_center_rgb, depth_value);
        // std::cout << "pixel_and_depth:" << pixel_and_depth << std::endl;
        Eigen::Vector3d point_w = transformPixel2World(pixel_and_depth);
        int type_class = 8;
        values.x = point_w.x();
        values.y = point_w.y();
        values.z = point_w.z();
        Eigen::Vector3d point_m(odom_->pose.pose.position.x,odom_->pose.pose.position.y,odom_->pose.pose.position.z);
        double d = (point_w - point_m).norm();
        values.d = d;
        values.class_val = type_class;
        cc = 1;
    }
    
}

TrajectoryPublisherNode::~TrajectoryPublisherNode() {}

void TrajectoryPublisherNode::boundingBoxes(const yolov8_ros_msgs::BoundingBoxesConstPtr &msg){
    if (!msg) {
        std::cerr << "BoundingBoxes message is null!" << std::endl;
        return;
    }
    yolo_ = msg;
    if(odom_pos_(2) <= -0.2 && stamp3 != yolo_->header.stamp){
        stamp3 = yolo_->header.stamp;
        Value data(Json::arrayValue);
        std::string type_class;
        for (const auto& box : yolo_->bounding_boxes) {
            Value obj;
            if(box.Class == "class_1"){
                type_class = "8";
            }  
            else if(box.Class == "class_0"){
                type_class = "8";
            }
            else{
                return;
            }
            obj["ObjType"] = type_class;
            obj["ULPointX"] = static_cast<double>(box.xmin); // xmin
            obj["ULPointY"] = static_cast<double>(box.ymin); // ymin
            obj["DRPointX"] = static_cast<double>(box.xmax); // xmax
            obj["DRPointY"] = static_cast<double>(box.ymax); // ymax                if(box.xmin)
            {
                std::cout << "obj contains: " << obj.toStyledString() << std::endl;
            }
            // 将边界框添加到数据数组中
            data.append(obj);
        }
        Value root;
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        root["timestamp"] = Json::Value(static_cast<Int64>(ms)); // 将uint64_t转换为Json::Value::UInt64类型 // 将时间戳转换为纳秒
        root["resX"] = resX;
        root["resY"] = resY;
        root["cameraName"] = cameraName;
        root["data"] = data;
        // 使用jsoncpp的StreamWriter来格式化JSON
        StreamWriterBuilder builder;
        builder["indentation"] = "\t"; // 使用制表符缩进
        const std::string json_str = Json::writeString(builder, root);
        if(data.empty()){
            return;
        }
        client.simUpdateLocalDetectTargetNumData(vehicle, json_str, waypoint_flag_data);

        // std::this_thread::sleep_for(std::chrono::milliseconds(11));  
    }
    client.simUpdateLocalDetectTargetPreData(vehicle, 6.25, -0.33, 0.0999, 8, waypoint_flag_data);
    client.simUpdateLocalTargetDistanceData(vehicle, values.d, values.x, values.y, values.z, 8, true, waypoint_flag_data);
}

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
    if(odom_pos_(2) <= 0 && stamp1 != odom_->header.stamp){
        stamp1 = odom_->header.stamp;
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
        client.simUpdateLocalPositionData(vehicle, odom_pos_(0), odom_pos_(1), odom_pos_(2), ms, 8, waypoint_flag_data);
        client.simUpdateLocalRotationData(vehicle, 1, 2, 3,4, ms, waypoint_flag_data);
        // cout << odom_pos_(0) << ", " << odom_pos_(1) << ", " << odom_pos_(2) << ", " << endl;
    }
    if(odom_ == nullptr)
        return;
    for (int i = waypoint_flag; i < waypoints_.rows(); ++i) {
        // 计算当前位置与waypoint点之间的距离     
        geometry_msgs::Point current_pos_ros;
        current_pos_ros.x = odom_->pose.pose.position.x;
        current_pos_ros.y = odom_->pose.pose.position.y;
        current_pos_ros.z = odom_->pose.pose.position.z;
        Eigen::Vector3d current_pos(current_pos_ros.x, current_pos_ros.y, current_pos_ros.z);
        Eigen::Vector3d waypoint = waypoints_.row(i);
        distance1 = (current_pos - waypoint).norm();       
        // 如果距离小于某个阈值，则认为无人机已到达该waypoint点
        if (distance1 < 10) {
            waypoint_flag++;
            std::string text = "Point_";
            text += std::to_string(waypoint_flag) + "_1";
            waypoint_flag_data = text;
            cout << waypoint_flag_data << endl;
            break;
        }
    }
}
void TrajectoryPublisherNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg){
    if(msg->latitude - gps_flag_data > 0.2){
        gps_flag = true;
        client.simUpdateLocalCheckNaviData(vehicle, gps_flag, waypoint_flag_data);
    }
    if(msg->latitude - gps_flag_data < 0.2){
        gps_flag = false;
        client.simUpdateLocalCheckNaviData(vehicle, gps_flag, waypoint_flag_data);
    }
    gps_flag_data = msg->latitude;
        // gps_flag = false;
        // client.simUpdateLocalCheckNaviData(vehicle, gps_flag, waypoint_flag_data);
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