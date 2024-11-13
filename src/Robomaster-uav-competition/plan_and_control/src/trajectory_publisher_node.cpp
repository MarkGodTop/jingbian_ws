#include "plan_and_control/trajectory_publisher_node.h"

msr::airlib::MultirotorRpcLibClient client("192.168.1.51");
TrajectoryPublisherNode::TrajectoryPublisherNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
:nh_(nh), nh_private_(nh_private)  {
    
    odom_sub_ =
        nh_.subscribe<nav_msgs::Odometry>("/airsim_node/UnmannedAirplane_1/odom_local_ned",1, &TrajectoryPublisherNode::odomCallback,this);
    yolo_sub_ = 
        nh_.subscribe<yolov8_ros_msgs::BoundingBoxes>("/yolov8/BoundingBoxes",10, &TrajectoryPublisherNode::boundingBoxes,this);
    gps_sub_ =
        nh_.subscribe<sensor_msgs::NavSatFix>("/airsim_node/UnmannedAirplane_1/gps/gps",1, &TrajectoryPublisherNode::gpsCallback,this);
    
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
            if(box.Class.empty())
            type_class = "0";
            if(box.Class == "class_1")
                type_class = "1";
            if(box.Class == "class_0")
                type_class = "8";
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
        client.simUpdateLocalDetectTargetNumData(vehicle, json_str);
        // std::this_thread::sleep_for(std::chrono::milliseconds(11));  
    }
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
    if(odom_pos_(2) <= -0.2 && stamp1 != odom_->header.stamp){
        stamp1 = odom_->header.stamp;
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();    
        client.simUpdateLocalPositionData(vehicle, odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z, ms, 8);
    }
}
void TrajectoryPublisherNode::gpsCallback(const sensor_msgs::NavSatFixConstPtr &msg){
    gps_flag_data = msg->latitude;
    if(msg->latitude - gps_flag_data > 0.5){
        gps_flag = true;
        client.simUpdateLocalCheckNaviData(vehicle, gps_flag, "Point_1_1");
    }else{
        gps_flag = false;
        client.simUpdateLocalCheckNaviData(vehicle, gps_flag, "Point_1_1");
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