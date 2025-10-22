// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

/**
 * ===================================================================
 * A-LOAM激光里程计模块 (laserOdometry.cpp)
 * ===================================================================
 * 
 * 【功能概述】
 * 本模块是A-LOAM系统的核心组件之一，负责：
 * 1. 接收来自scanRegistration模块的特征点云（角点和平面点）
 * 2. 通过帧间特征匹配估计相机的相对运动
 * 3. 使用Ceres优化求解器进行非线性优化
 * 4. 发布激光里程计位姿和轨迹信息
 * 5. 为laserMapping模块提供初始位姿估计和特征点云
 * 
 * 【算法原理】
 * - 特征匹配：使用KD树在上一帧特征点中搜索当前帧特征点的最近邻
 * - 约束构建：
 *   * 角点约束：点到线的距离最小化
 *   * 平面点约束：点到面的距离最小化
 * - 位姿优化：使用Ceres求解器优化6自由度相对变换（3旋转+3平移）
 * - 畸变校正：考虑激光雷达扫描过程中载体运动造成的点云畸变
 * 
 * 【数据流】
 * scanRegistration → laserOdometry → laserMapping
 * 
 * 【关键变量说明】
 * - q_w_curr, t_w_curr: 当前帧相对于世界坐标系的位姿
 * - para_q, para_t: 优化变量（上一帧到当前帧的相对变换）
 * - kdtreeCornerLast, kdtreeSurfLast: 用于最近邻搜索的KD树
 * ===================================================================
 */

// 是否考虑点云畸变：0=不考虑，1=考虑
// 点云畸变是由于激光雷达旋转过程中载体运动造成的
#define DISTORTION 0

// 全局变量：记录特征对应关系的数量
int corner_correspondence = 0, plane_correspondence = 0;  // 角点和平面点的对应关系数量

// 关键参数定义
constexpr double SCAN_PERIOD = 0.1;                    // 激光雷达扫描周期：0.1秒（10Hz）
constexpr double DISTANCE_SQ_THRESHOLD = 25;           // 最近邻搜索的距离平方阈值：25（即5米）
constexpr double NEARBY_SCAN = 2.5;                    // 附近扫描线的搜索范围

// 系统参数
int skipFrameNum = 5;                                   // 跳过的帧数，用于降低建图模块的频率
bool systemInited = false;                              // 系统初始化标志

// 各种特征点云的时间戳，用于数据同步
double timeCornerPointsSharp = 0;                      // 强角点的时间戳
double timeCornerPointsLessSharp = 0;                  // 弱角点的时间戳  
double timeSurfPointsFlat = 0;                         // 强平面点的时间戳
double timeSurfPointsLessFlat = 0;                     // 弱平面点的时间戳
double timeLaserCloudFullRes = 0;                      // 完整点云的时间戳

// KD树：用于快速最近邻搜索
// 存储上一帧的特征点，用于当前帧特征点的匹配
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());  // 角点KD树
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());    // 平面点KD树

// === 当前帧的特征点云 ===
// 从scanRegistration模块接收到的当前帧特征点
pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());      // 当前帧强角点
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());  // 当前帧弱角点
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());         // 当前帧强平面点
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());     // 当前帧弱平面点

// === 上一帧的特征点云 ===
// 用于与当前帧进行特征匹配的参考点云
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());   // 上一帧角点
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());     // 上一帧平面点
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());      // 当前帧完整点云

// 上一帧特征点的数量统计
int laserCloudCornerLastNum = 0;                       // 上一帧角点数量
int laserCloudSurfLastNum = 0;                         // 上一帧平面点数量

// === 变换矩阵存储 ===
// 当前帧相对于世界坐标系的变换
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);              // 当前帧相对于世界坐标系的四元数旋转
Eigen::Vector3d t_w_curr(0, 0, 0);                    // 当前帧相对于世界坐标系的平移向量

// 优化求解的参数变量（6自由度：3旋转+3平移）
// 表示上一帧到当前帧的相对变换
double para_q[4] = {0, 0, 0, 1};                      // 四元数参数 [qx, qy, qz, qw]
double para_t[3] = {0, 0, 0};                         // 平移参数 [tx, ty, tz]

// === 特征点变换相关变量 ===
// 用Eigen::Map将数组映射为Eigen类型，方便矩阵运算
Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);   // 上一帧到当前帧的四元数变换
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);      // 上一帧到当前帧的平移变换

// === 消息缓冲队列 ===
// 用于缓存来自scanRegistration节点的特征点云消息，保证时序同步
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf;      // 强角点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf;  // 弱角点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf;         // 强平面点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf;     // 弱平面点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf;       // 完整点云消息缓冲
std::mutex mBuf;                                                   // 缓冲队列的互斥锁，保证线程安全

// === 点云去畸变函数 ===
// 将点云从扫描结束时刻变换到扫描开始时刻，消除运动畸变
void TransformToStart(PointType const *const pi, PointType *const po)
{
    // 计算插值比例s，表示该点在整个扫描周期中的相对时间位置
    double s;
    if (DISTORTION)
        // 从intensity字段中提取时间信息：小数部分表示相对时间
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;  // 不考虑畸变时，直接使用完整变换
    // 计算该点对应的旋转变换：使用球面线性插值(slerp)
    // Identity()表示无旋转，slerp(s, q_last_curr)表示从无旋转插值到完整旋转
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    
    // 计算该点对应的平移变换：线性插值
    Eigen::Vector3d t_point_last = s * t_last_curr;
    
    // 将输入点转为Eigen向量格式
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    
    // 应用变换：先旋转后平移，得到去畸变后的点
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    // 将结果赋值给输出点
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;  // 保持原始强度信息
}

// === 点云变换到扫描结束时刻函数 ===
// 将所有激光点变换到下一帧的开始时刻（即当前帧的结束时刻）
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // 第一步：先对点进行去畸变处理，变换到扫描开始时刻
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    // 第二步：将去畸变后的点从扫描开始时刻变换到扫描结束时刻
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    
    // 应用逆变换：先减去平移，再应用逆旋转
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    // 将结果赋值给输出点
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    // 移除畸变时间信息：只保留扫描线ID，去掉小数部分的时间戳
    po->intensity = int(pi->intensity);
}

// === 特征点云消息回调函数 ===
// 接收来自scanRegistration节点的各类特征点云消息并缓存

// 强角点消息回调函数
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();                    // 加锁保护缓冲队列
    cornerSharpBuf.push(cornerPointsSharp2);  // 将消息添加到缓冲队列
    mBuf.unlock();                  // 解锁
}

// 弱角点消息回调函数
void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();
    cornerLessSharpBuf.push(cornerPointsLessSharp2);
    mBuf.unlock();
}

// 强平面点消息回调函数
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();
    surfFlatBuf.push(surfPointsFlat2);
    mBuf.unlock();
}

// 弱平面点消息回调函数
void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();
    surfLessFlatBuf.push(surfPointsLessFlat2);
    mBuf.unlock();
}

// 完整点云消息回调函数
// 接收所有激光点（用于最终的地图构建和可视化）
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();
    fullPointsBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

// === 主函数 ===
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    // 读取参数：跳帧数量（用于降低地图构建频率）
    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);
    printf("Mapping %d Hz \n", 10 / skipFrameNum);

    // === 订阅器设置 ===
    // 订阅来自scanRegistration节点的各类特征点云
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_sharp", 100, laserCloudSharpHandler);                    // 强角点

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);           // 弱角点

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_flat", 100, laserCloudFlatHandler);                      // 强平面点

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);             // 弱平面点

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>(
        "/velodyne_cloud_2", 100, laserCloudFullResHandler);                   // 完整点云

    // === 发布器设置 ===
    // 发布处理后的点云和位姿信息给laserMapping节点
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>(
        "/laser_cloud_corner_last", 100);      // 上一帧角点（给mapping用）

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>(
        "/laser_cloud_surf_last", 100);        // 上一帧平面点（给mapping用）

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>(
        "/velodyne_cloud_3", 100);             // 去畸变后的完整点云

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>(
        "/laser_odom_to_init", 100);           // 激光里程计位姿

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>(
        "/laser_odom_path", 100);              // 激光里程计轨迹

    // 轨迹路径消息
    nav_msgs::Path laserPath;

    // 帧计数器和循环频率控制
    int frameCount = 0;
    ros::Rate rate(100);       // 100Hz主循环

    // === 主处理循环 ===
    while (ros::ok())
    {
        ros::spinOnce();  // 处理回调函数

        // 检查所有消息缓冲队列是否都有数据
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())
        {
            // === 时间戳同步检查 ===
            // 获取各个消息的时间戳，确保来自同一帧
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();

            // 检查时间戳是否一致，确保数据同步（允许一定容差并丢弃旧数据）
            const double timeTolerance = 1e-3; // 1ms容差
            const double timeMin = std::min(
                {timeCornerPointsSharp, timeCornerPointsLessSharp,
                 timeSurfPointsFlat, timeSurfPointsLessFlat, timeLaserCloudFullRes});
            const double timeMax = std::max(
                {timeCornerPointsSharp, timeCornerPointsLessSharp,
                 timeSurfPointsFlat, timeSurfPointsLessFlat, timeLaserCloudFullRes});

            if (timeMax - timeMin > timeTolerance)
            {
                ROS_WARN_STREAM_THROTTLE(1.0,
                    "laserOdometry: unsync message ("
                    << std::fixed << std::setprecision(6)
                    << "corner_sharp=" << timeCornerPointsSharp
                    << ", corner_less=" << timeCornerPointsLessSharp
                    << ", surf_flat=" << timeSurfPointsFlat
                    << ", surf_less=" << timeSurfPointsLessFlat
                    << ", full=" << timeLaserCloudFullRes << "). Drop oldest data.");

                auto nearlyEqual = [](double a, double b) {
                    return std::fabs(a - b) < 1e-6;
                };

                bool dropped = false;
                mBuf.lock();
                if (!cornerSharpBuf.empty() && nearlyEqual(cornerSharpBuf.front()->header.stamp.toSec(), timeMin))
                {
                    cornerSharpBuf.pop();
                    dropped = true;
                }
                if (!dropped && !cornerLessSharpBuf.empty() && nearlyEqual(cornerLessSharpBuf.front()->header.stamp.toSec(), timeMin))
                {
                    cornerLessSharpBuf.pop();
                    dropped = true;
                }
                if (!dropped && !surfFlatBuf.empty() && nearlyEqual(surfFlatBuf.front()->header.stamp.toSec(), timeMin))
                {
                    surfFlatBuf.pop();
                    dropped = true;
                }
                if (!dropped && !surfLessFlatBuf.empty() && nearlyEqual(surfLessFlatBuf.front()->header.stamp.toSec(), timeMin))
                {
                    surfLessFlatBuf.pop();
                    dropped = true;
                }
                if (!dropped && !fullPointsBuf.empty() && nearlyEqual(fullPointsBuf.front()->header.stamp.toSec(), timeMin))
                {
                    fullPointsBuf.pop();
                    dropped = true;
                }
                if (!dropped && !fullPointsBuf.empty())
                {
                    // 若无法定位最旧的队列，则保守地弹出完整点云队列的首元素
                    fullPointsBuf.pop();
                }
                mBuf.unlock();
                continue;
            }

            // === 数据提取与转换 ===
            // 从消息缓冲队列中提取数据并转换为PCL格式
            mBuf.lock();  // 加锁保护缓冲队列
            
            // 提取强角点
            cornerPointsSharp->clear();
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
            cornerSharpBuf.pop();

            // 提取弱角点
            cornerPointsLessSharp->clear();
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();

            // 提取强平面点
            surfPointsFlat->clear();
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();

            // 提取弱平面点
            surfPointsLessFlat->clear();
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();

            // 提取完整点云
            laserCloudFullRes->clear();
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            
            mBuf.unlock();  // 解锁

            // === 开始处理计时 ===
            TicToc t_whole;
            
            // === 系统初始化检查 ===
            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else  // 系统已初始化，开始正常处理
            {
                // 获取当前帧特征点数量
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                // === 迭代优化过程 ===
                // 进行两次优化迭代以提高精度
                TicToc t_opt;
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    // 重置对应关系计数器
                    corner_correspondence = 0;    // 角点对应关系数量
                    plane_correspondence = 0;     // 平面点对应关系数量

                    // === Ceres优化器设置 ===
                    // 使用Huber损失函数，降低离群点影响
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    
                    // 四元数参数化：保证四元数的约束（单位长度）
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    
                    // 创建优化问题
                    ceres::Problem::Options problem_options;
                    ceres::Problem problem(problem_options);
                    
                    // 添加参数块：4个四元数参数和3个平移参数
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    // 用于存储变换后的点和搜索结果
                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;        // 搜索到的点索引
                    std::vector<float> pointSearchSqDis;    // 搜索到的点距离平方

                    // === 角点特征匹配 ===
                    TicToc t_data;
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        // 将当前帧角点变换到扫描开始时刻
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        
                        // 在上一帧角点中搜索最近邻点
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        
                        // 检查最近邻点是否足够近
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            // 获取最近邻点的索引和扫描线ID
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            
                            // === 向上搜索：在扫描线ID递增方向寻找第二个最近点 ===
                            // 目标：找到与当前点共线的另一个点，构成线段约束
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // 跳过相同扫描线上的点
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // 如果超出附近扫描线范围，停止搜索
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                // 计算候选点到当前点的欧氏距离平方
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                // 如果找到更近的点，更新最近距离和索引
                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // === 向下搜索：在扫描线ID递减方向寻找第二个最近点 ===
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // 跳过相同扫描线上的点
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // 如果超出附近扫描线范围，停止搜索
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                // 计算候选点到当前点的欧氏距离平方
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                // 如果找到更近的点，更新最近距离和索引
                                if (pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        
                        // === 构建角点约束 ===
                        // 如果成功找到两个匹配点，构建点到线的距离约束
                        if (minPointInd2 >= 0) // 确保找到了有效的第二个匹配点
                        {
                            // 当前帧角点坐标
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            // 上一帧中的第一个匹配点                               
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            // 上一帧中的第二个匹配点
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);

                            // 计算畸变系数s：表示该点在扫描周期中的相对时间位置
                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;  // 不考虑畸变
                            
                            // 创建角点-线段匹配的代价函数：点到线距离最小化
                            // LidarEdgeFactor实现点到线段的距离计算和雅可比矩阵
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;  // 角点对应关系计数增加
                        }
                    }

                    // === 平面点特征匹配 ===
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        // 将当前帧平面点变换到扫描开始时刻
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        // 在上一帧平面点中搜索最近邻点
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        // 用于存储找到的三个匹配点索引（构成平面需要三个点）
                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        
                        // 检查最近邻点是否足够近
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // 获取最近点的扫描线ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // === 向上搜索：寻找构成平面的另外两个点 ===
                            // 需要找到分别位于同一扫描线和不同扫描线上的两个点
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // 如果超出附近扫描线范围，停止搜索
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                // 计算候选点到当前点的欧氏距离平方
                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 在相同或更低扫描线上寻找第二个点
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // 在更高扫描线上寻找第三个点
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // === 向下搜索：继续寻找构成平面的另外两个点 ===
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // 如果超出附近扫描线范围，停止搜索
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                // 计算候选点到当前点的欧氏距离平方
                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 在相同或更高扫描线上寻找第二个点
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // 在更低扫描线上寻找第三个点
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // === 构建平面点约束 ===
                            // 如果成功找到三个匹配点，构建点到面的距离约束
                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {
                                // 当前帧平面点坐标
                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                // 上一帧中构成平面的三个点
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                // 计算畸变系数s：表示该点在扫描周期中的相对时间位置
                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;  // 不考虑畸变
                                
                                // 创建平面点-平面匹配的代价函数：点到平面距离最小化
                                // LidarPlaneFactor实现点到三角平面的距离计算和雅可比矩阵
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;  // 平面点对应关系计数增加
                            }
                        }
                    }

                    // === 数据关联统计输出 ===
                    printf("data association time %f ms \n", t_data.toc());

                    // 检查对应关系数量：如果太少可能影响优化效果
                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    // === Ceres求解器配置和优化 ===
                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;      // 使用稠密QR分解求解线性方程组
                    options.max_num_iterations = 4;                   // 最大迭代次数：4次（实时性考虑）
                    options.minimizer_progress_to_stdout = false;     // 不输出优化过程信息
                    
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);        // 执行非线性优化
                    
                    // 输出求解器耗时
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());

                // === 更新全局变换 ===
                // 将帧间相对变换累积到全局位姿中
                t_w_curr = t_w_curr + q_w_curr * t_last_curr;  // 更新全局平移：t_new = t_old + R_old * delta_t
                q_w_curr = q_w_curr * q_last_curr;             // 更新全局旋转：R_new = R_old * delta_R
            }

            // === 结果发布 ===
            TicToc t_pub;

            // 发布激光里程计位姿信息
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";     // 参考坐标系
            laserOdometry.child_frame_id = "/laser_odom";       // 激光里程计坐标系
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            
            // 设置四元数旋转
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            
            // 设置位置平移
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            // 发布激光里程计轨迹路径
            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);      // 添加当前位姿到轨迹
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

            // === 点云变换到扫描结束时刻 ===
            // （这部分代码被注释掉，通常不执行）
            if (0)
            {
                // 变换弱角点到扫描结束时刻
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                // 变换弱平面点到扫描结束时刻
                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                // 变换完整点云到扫描结束时刻
                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            // === 点云数据更新 ===
            // 交换当前帧和上一帧的点云指针，为下一帧处理做准备
            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;      // 当前帧弱角点 -> 上一帧角点
            laserCloudCornerLast = laserCloudTemp;             // 上一帧角点 -> 当前帧弱角点

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;           // 当前帧弱平面点 -> 上一帧平面点
            laserCloudSurfLast = laserCloudTemp;               // 上一帧平面点 -> 当前帧弱平面点

            // 更新点云数量
            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // === KD树更新 ===
            // 为下一帧匹配准备KD树
            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);   // 重建角点KD树
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);       // 重建平面点KD树

            // === 选择性发布数据给laserMapping ===
            // 按照skipFrameNum跳帧发布，降低地图构建频率
            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                // 发布角点给laserMapping节点
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                // 发布平面点给laserMapping节点
                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                // 发布完整点云给laserMapping节点
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            
            // === 性能统计输出 ===
            printf("publication time %f ms \n", t_pub.toc());                    // 发布耗时
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());       // 总耗时
            
            // 警告：如果处理时间超过100ms，可能影响实时性
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;  // 帧计数器递增
        }
        rate.sleep();      // 保持循环频率
    }
    return 0;
}
