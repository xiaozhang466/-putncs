/**
 * @file kittiHelper.cpp
 * @brief KITTI数据集助手程序
 * @description 读取KITTI数据集并将其转换为ROS消息格式，支持实时发布和保存为rosbag
 * @author Tong Qin (qintonguav@gmail.com), Shaozu Cao (saozu.cao@connect.ust.hk)
 */

// 标准库头文件
#include <iostream>        // 标准输入输出流
#include <fstream>         // 文件流操作
#include <iterator>        // 迭代器支持
#include <string>          // 字符串操作
#include <vector>          // 动态数组容器

// OpenCV相关头文件
#include <opencv2/opencv.hpp>              // OpenCV主头文件
#include <image_transport/image_transport.h>  // ROS图像传输
#include <opencv2/highgui/highgui.hpp>     // OpenCV图像显示和IO

// ROS消息类型头文件
#include <nav_msgs/Odometry.h>        // 里程计消息
#include <nav_msgs/Path.h>            // 路径消息
#include <geometry_msgs/PoseStamped.h>  // 带时间戳的位姿消息

// ROS核心头文件
#include <ros/ros.h>           // ROS核心功能
#include <rosbag/bag.h>        // ROS bag文件操作

// CV Bridge和传感器消息
#include <cv_bridge/cv_bridge.h>         // OpenCV与ROS图像消息转换
#include <sensor_msgs/image_encodings.h>  // 图像编码类型
#include <sensor_msgs/PointCloud2.h>     // 点云消息类型

// 第三方库头文件
#include <eigen3/Eigen/Dense>        // Eigen矩阵库
#include <pcl/point_cloud.h>         // PCL点云库
#include <pcl/point_types.h>         // PCL点类型定义
#include <pcl_conversions/pcl_conversions.h>  // PCL与ROS消息转换

/**
 * @brief 读取KITTI激光雷达数据文件
 * @param lidar_data_path 激光雷达数据文件路径
 * @return 返回包含激光雷达数据的float向量
 * @description KITTI数据集中激光雷达数据以二进制格式存储，每个点包含4个float值(x,y,z,intensity)
 */
std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    // 以二进制模式打开激光雷达数据文件
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    
    // 定位到文件末尾以获取文件大小
    lidar_data_file.seekg(0, std::ios::end);
    
    // 计算float元素的总数（文件大小除以float大小）
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    
    // 重新定位到文件开头
    lidar_data_file.seekg(0, std::ios::beg);

    // 创建缓冲区存储激光雷达数据
    std::vector<float> lidar_data_buffer(num_elements);
    
    // 读取整个文件到缓冲区
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    
    return lidar_data_buffer;
}

/**
 * @brief 主函数 - KITTI数据集处理和ROS消息发布
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @description 读取KITTI数据集，包括图像、激光雷达点云和真实轨迹，转换为ROS消息并发布
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点，节点名为"kitti_helper"
    ros::init(argc, argv, "kitti_helper");
    
    // 创建私有节点句柄，用于获取参数
    ros::NodeHandle n("~");
    
    // ========== 获取ROS参数 ==========
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);      // KITTI数据集根目录
    n.getParam("sequence_number", sequence_number);    // 序列号（如00, 01, 02等）
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    
    // 是否保存为rosbag文件
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);  // 输出bag文件路径
    
    // 发布延迟参数（控制播放速度）
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    // ========== 创建ROS发布者 ==========
    // 激光雷达点云发布者
    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    // 图像传输对象和发布者
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);   // 左目图像发布者
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2); // 右目图像发布者

    // 真实轨迹里程计发布者
    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";  // 参考坐标系
    odomGT.child_frame_id = "/ground_truth";  // 子坐标系

    // 真实轨迹路径发布者  
    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";  // 路径参考坐标系

    // ========== 打开数据文件 ==========
    // 时间戳文件路径和文件流
    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    // 真实轨迹文件路径和文件流
    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    // 如果需要保存为bag文件，则创建bag对象
    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    
    // ========== 坐标系转换矩阵 ==========
    // KITTI坐标系到ROS坐标系的转换矩阵
    // KITTI: x-forward, y-left, z-up
    // ROS: x-forward, y-left, z-up (实际上这里做了一个特殊的转换)
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    // ========== 数据处理循环 ==========
    std::string line;
    std::size_t line_num = 0;  // 当前处理的帧编号

    // 设置发布频率（考虑延迟参数）
    ros::Rate r(10.0 / publish_delay);
    
    // 主循环：逐帧读取时间戳并处理对应数据
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        // ========== 解析时间戳 ==========
        float timestamp = stof(line);  // 将字符串转换为浮点数时间戳
        
        // ========== 读取左右目图像 ==========
        std::stringstream left_image_path, right_image_path;
        // 构建左目图像路径（image_0目录）
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
        
        // 构建右目图像路径（image_1目录）
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" 
                         << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);  // 注意：这里原代码有bug，应该用right_image_path

        // ========== 读取真实轨迹数据 ==========
        std::getline(ground_truth_file, line);  // 读取当前帧对应的真实位姿
        std::stringstream pose_stream(line);
        std::string s;
        
        // 解析3x4的位姿矩阵 [R|t]
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');  // 以空格为分隔符读取
                gt_pose(i, j) = stof(s);
            }
        }

        // ========== 坐标系转换 ==========
        // 提取旋转矩阵并转换为四元数
        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;  // 应用坐标系转换
        q.normalize();  // 四元数归一化
        
        // 提取平移向量并应用坐标系转换
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        // ========== 构建并发布里程计消息 ==========
        odomGT.header.stamp = ros::Time().fromSec(timestamp);  // 设置时间戳
        // 设置方向（四元数）
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        // 设置位置
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);  // 发布里程计消息

        // ========== 构建并发布路径消息 ==========
        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;      // 复制头信息
        poseGT.pose = odomGT.pose.pose;     // 复制位姿信息
        pathGT.header.stamp = odomGT.header.stamp;  // 更新路径时间戳
        pathGT.poses.push_back(poseGT);     // 添加到路径中
        pubPathGT.publish(pathGT);          // 发布路径消息

        // ========== 读取激光雷达点云数据 ==========
        std::stringstream lidar_data_path;
        // 构建激光雷达数据文件路径
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        
        // 读取二进制激光雷达数据
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        // ========== 处理点云数据 ==========
        std::vector<Eigen::Vector3d> lidar_points;    // 存储点坐标
        std::vector<float> lidar_intensities;         // 存储点强度
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;  // PCL点云对象
        
        // 解析激光雷达数据（每4个float为一个点：x, y, z, intensity）
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            // 存储点坐标和强度到向量
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            // 构建PCL点并添加到点云
            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        // ========== 发布点云和图像消息 ==========
        // 将PCL点云转换为ROS消息格式
        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);  // 设置时间戳
        laser_cloud_msg.header.frame_id = "/camera_init";               // 设置坐标系
        pub_laser_cloud.publish(laser_cloud_msg);  // 发布点云消息

        // 将OpenCV图像转换为ROS图像消息并发布
        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);   // 发布左目图像
        pub_image_right.publish(image_right_msg); // 发布右目图像

        // ========== 保存到rosbag文件（可选） ==========
        if (to_bag)
        {
            // 将所有消息写入bag文件，使用当前时间作为时间戳
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        // 更新帧编号并控制发布频率
        line_num ++;        // 帧编号递增
        r.sleep();          // 按设定频率休眠
    }
    
    // ========== 清理和退出 ==========
    bag_out.close();  // 关闭bag文件
    std::cout << "Done \n";

    return 0;  // 程序正常退出
}