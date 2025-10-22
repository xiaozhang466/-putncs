/*
 * ===================================================================
 * 【A-LOAM激光建图模块 - laserMapping.cpp】
 * ===================================================================
 * 
 * 【功能描述】
 * 本模块是A-LOAM系统的核心建图组件，负责：
 * 1. 接收来自laserOdometry的位姿估计和特征点云数据
 * 2. 构建和维护全局3D点云地图
 * 3. 通过地图约束优化机器人的全局位姿
 * 4. 发布优化后的位姿、轨迹和地图信息
 * 
 * 【算法原理】
 * - 基于滑动窗口的局部地图维护策略
 * - 使用体素网格进行高效的点云存储和检索
 * - 通过Ceres优化器进行点到线/面的距离最小化
 * - 采用KD-Tree加速最近邻搜索
 * 
 * 【数据流】
 * laserOdometry → laserMapping → 全局地图 + 优化位姿
 * 
 * 【坐标系说明】
 * - /camera_init: 全局地图坐标系（世界坐标系）
 * - /camera: 当前帧坐标系（来自里程计）
 * - /aft_mapped: 建图后优化的坐标系
 * ===================================================================
 */

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
#include <iomanip>
#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

// ===================================================================
// 【全局变量定义】
// ===================================================================

int frameCount = 0;                                    // 帧计数器，用于控制发布频率

// === 时间戳变量 - 用于数据同步 ===
double timeLaserCloudCornerLast = 0;                   // 角点点云的时间戳
double timeLaserCloudSurfLast = 0;                     // 平面点点云的时间戳
double timeLaserCloudFullRes = 0;                      // 完整点云的时间戳
double timeLaserOdometry = 0;                          // 里程计的时间戳

// === 3D体素网格参数 - 用于管理局部地图 ===
// 地图被分割为21x21x11的3D网格，每个网格存储50x50x50米的空间
int laserCloudCenWidth = 10;                          // 中心网格的宽度索引（X轴）
int laserCloudCenHeight = 10;                         // 中心网格的高度索引（Y轴）
int laserCloudCenDepth = 5;                           // 中心网格的深度索引（Z轴）
const int laserCloudWidth = 21;                      // 网格总宽度（X轴方向）
const int laserCloudHeight = 21;                     // 网格总高度（Y轴方向）
const int laserCloudDepth = 11;                      // 网格总深度（Z轴方向）

// 总的网格数量：21x21x11 = 4851个立方体
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

// === 网格索引数组 ===
int laserCloudValidInd[125];                         // 有效网格索引（5x5x5=125个）
int laserCloudSurroundInd[125];                      // 周围网格索引，用于可视化

// === 输入点云数据 - 来自里程计模块 ===
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());   // 角点特征
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());     // 平面点特征

// === 输出点云数据 - 用于可视化 ===
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());     // 周围可见的立方体点云

// === 建图优化用的局部地图点云 ===
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>()); // 从地图中提取的角点
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());   // 从地图中提取的平面点

// === 完整点云数据 - 用于建图 ===
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());      // 输入输出：单帧点云 local->global

// === 3D网格地图存储 - 每个网格存储角点和平面点 ===
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];                     // 存储每个网格的角点
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];                       // 存储每个网格的平面点

// === KD树 - 用于快速最近邻搜索 ===
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());  // 角点KD树
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());    // 平面点KD树

// === 位姿优化参数 - 使用Ceres求解器优化 ===
double parameters[7] = {0, 0, 0, 1, 0, 0, 0};        // 7个参数：[qx, qy, qz, qw, tx, ty, tz]
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);  // 当前帧相对于地图世界坐标系的旋转
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4); // 当前帧相对于地图世界坐标系的平移

// === 坐标系变换关系 ===
// 变换关系：wmap_T_odom * odom_T_curr = wmap_T_curr
// 地图世界坐标系和里程计世界坐标系之间的变换
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);          // 地图世界到里程计世界的旋转变换
Eigen::Vector3d t_wmap_wodom(0, 0, 0);                // 地图世界到里程计世界的平移变换

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);          // 当前帧相对于里程计世界坐标系的旋转
Eigen::Vector3d t_wodom_curr(0, 0, 0);                // 当前帧相对于里程计世界坐标系的平移

// === 消息缓冲队列 - 确保数据时序同步 ===
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;  // 角点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;    // 平面点消息缓冲
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;     // 完整点云消息缓冲
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;        // 里程计消息缓冲
std::mutex mBuf;                                              // 缓冲队列互斥锁

// === 体素滤波器 - 降采样以提高效率 ===
pcl::VoxelGrid<PointType> downSizeFilterCorner;              // 角点降采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterSurf;                // 平面点降采样滤波器

// === 搜索结果存储 ===
std::vector<int> pointSearchInd;                             // 最近邻点的索引
std::vector<float> pointSearchSqDis;                         // 最近邻点的距离平方

PointType pointOri, pointSel;                                // 原始点和选择点

// === ROS发布器 ===
ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

nav_msgs::Path laserAfterMappedPath;                          // 建图后的轨迹路径

// ===================================================================
// 【坐标变换函数】
// ===================================================================

// 设置初始猜测值：将里程计位姿变换到地图坐标系
void transformAssociateToMap()
{
	// 计算当前帧在地图坐标系中的位姿
	// T_w_curr = T_wmap_wodom * T_wodom_curr
	q_w_curr = q_wmap_wodom * q_wodom_curr;                    // 旋转复合
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;     // 位移复合
}

// 更新地图世界坐标系与里程计世界坐标系之间的变换
void transformUpdate()
{
	// 计算变换关系：T_wmap_wodom = T_w_curr * T_wodom_curr^(-1)
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();          // 更新旋转变换
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;     // 更新平移变换
}

// 将点从当前帧坐标系变换到地图世界坐标系
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);          // 当前帧坐标系下的点
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr; // 变换到地图世界坐标系
	po->x = point_w.x();                                       // 赋值X坐标
	po->y = point_w.y();                                       // 赋值Y坐标
	po->z = point_w.z();                                       // 赋值Z坐标
	po->intensity = pi->intensity;                             // 保持强度不变
	//po->intensity = 1.0;
}

// 将点从地图世界坐标系变换到当前帧坐标系（与上面函数功能相反）
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);             // 地图世界坐标系下的点
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr); // 逆变换到当前帧坐标系
	po->x = point_curr.x();                                    // 赋值X坐标
	po->y = point_curr.y();                                    // 赋值Y坐标
	po->z = point_curr.z();                                    // 赋值Z坐标
	po->intensity = pi->intensity;                             // 保持强度不变
}

// ===================================================================
// 【消息回调函数】
// ===================================================================

// 角点点云消息回调函数
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
	mBuf.lock();                                               // 加锁保护缓冲队列
	cornerLastBuf.push(laserCloudCornerLast2);                 // 将消息推入缓冲队列
	mBuf.unlock();                                             // 解锁
}

// 平面点点云消息回调函数
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
	mBuf.lock();                                               // 加锁保护缓冲队列
	surfLastBuf.push(laserCloudSurfLast2);                     // 将消息推入缓冲队列
	mBuf.unlock();                                             // 解锁
}

// 完整点云消息回调函数
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	mBuf.lock();                                               // 加锁保护缓冲队列
	fullResBuf.push(laserCloudFullRes2);                       // 将消息推入缓冲队列
	mBuf.unlock();                                             // 解锁
}

// 里程计消息回调函数
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	mBuf.lock();                                               // 加锁保护缓冲队列
	odometryBuf.push(laserOdometry);                           // 将里程计消息推入缓冲队列
	mBuf.unlock();                                             // 解锁

	// === 高频发布优化后位姿 ===
	// 从里程计消息中提取位姿信息
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x; // 提取四元数X分量
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y; // 提取四元数Y分量
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z; // 提取四元数Z分量
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w; // 提取四元数W分量
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;    // 提取位置X坐标
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;    // 提取位置Y坐标
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;    // 提取位置Z坐标

	// 将里程计位姿变换到地图世界坐标系
	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	// 构造建图后的里程计消息
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";            // 参考坐标系：地图世界坐标系
	odomAftMapped.child_frame_id = "/aft_mapped";              // 子坐标系：建图后坐标系
	odomAftMapped.header.stamp = laserOdometry->header.stamp;  // 时间戳
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();      // 设置四元数X分量
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();      // 设置四元数Y分量
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();      // 设置四元数Z分量
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();      // 设置四元数W分量
	odomAftMapped.pose.pose.position.x = t_w_curr.x();         // 设置位置X坐标
	odomAftMapped.pose.pose.position.y = t_w_curr.y();         // 设置位置Y坐标
	odomAftMapped.pose.pose.position.z = t_w_curr.z();         // 设置位置Z坐标
	pubOdomAftMappedHighFrec.publish(odomAftMapped);            // 发布高频优化位姿
}

// ===================================================================
// 【主处理函数 - 建图核心算法】
// ===================================================================
void process()
{
	while(1)
	{
		// === 等待所有必需数据就绪 ===
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
			!fullResBuf.empty() && !odometryBuf.empty())
		{
			mBuf.lock();
			
			// 时间同步：确保里程计时间戳不早于角点时间戳
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				odometryBuf.pop();
			if (odometryBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// 时间同步：确保平面点时间戳不早于角点时间戳
			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();
			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// 时间同步：确保完整点云时间戳不早于角点时间戳
			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				fullResBuf.pop();
			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// === 提取时间戳并检查同步 ===
			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();  // 角点时间戳
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();      // 平面点时间戳
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();        // 完整点云时间戳
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();           // 里程计时间戳

			// 检查时间戳是否同步（允许一定容差并丢弃旧数据）
			const double timeTolerance = 1e-3; // 1ms容差
			const double timeMin = std::min(
				{timeLaserCloudCornerLast, timeLaserCloudSurfLast,
				 timeLaserCloudFullRes, timeLaserOdometry});
			const double timeMax = std::max(
				{timeLaserCloudCornerLast, timeLaserCloudSurfLast,
				 timeLaserCloudFullRes, timeLaserOdometry});

			if (timeMax - timeMin > timeTolerance)
			{
				ROS_WARN_STREAM_THROTTLE(1.0,
					"laserMapping: unsync message ("
					<< std::fixed << std::setprecision(6)
					<< "corner=" << timeLaserCloudCornerLast
					<< ", surf=" << timeLaserCloudSurfLast
					<< ", full=" << timeLaserCloudFullRes
					<< ", odom=" << timeLaserOdometry
					<< "). Drop oldest data.");

				auto nearlyEqual = [](double a, double b) {
					return std::fabs(a - b) < 1e-6;
				};

				bool dropped = false;
				if (!cornerLastBuf.empty() && nearlyEqual(cornerLastBuf.front()->header.stamp.toSec(), timeMin))
				{
					cornerLastBuf.pop();
					dropped = true;
				}
				if (!dropped && !surfLastBuf.empty() && nearlyEqual(surfLastBuf.front()->header.stamp.toSec(), timeMin))
				{
					surfLastBuf.pop();
					dropped = true;
				}
				if (!dropped && !fullResBuf.empty() && nearlyEqual(fullResBuf.front()->header.stamp.toSec(), timeMin))
				{
					fullResBuf.pop();
					dropped = true;
				}
				if (!dropped && !odometryBuf.empty() && nearlyEqual(odometryBuf.front()->header.stamp.toSec(), timeMin))
				{
					odometryBuf.pop();
					dropped = true;
				}
				if (!dropped && !fullResBuf.empty())
				{
					fullResBuf.pop();
				}

				mBuf.unlock();
				continue;
			}

			// === 提取点云数据 ===
			laserCloudCornerLast->clear();                        // 清空角点容器
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast); // ROS消息转PCL格式
			cornerLastBuf.pop();                                  // 弹出已处理的消息

			laserCloudSurfLast->clear();                              // 清空平面点容器
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast); // ROS消息转PCL格式
			surfLastBuf.pop();                                    // 弹出已处理的消息

			laserCloudFullRes->clear();                           // 清空完整点云容器
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes); // ROS消息转PCL格式
			fullResBuf.pop();                                     // 弹出已处理的消息

			// === 提取里程计位姿数据 ===
			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x; // 四元数X分量
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y; // 四元数Y分量
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z; // 四元数Z分量
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w; // 四元数W分量
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;    // 位置X坐标
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;    // 位置Y坐标
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;    // 位置Z坐标
			odometryBuf.pop();                                    // 弹出已处理的消息

			// === 丢弃多余帧以保证实时性能 ===
			while(!cornerLastBuf.empty())
			{
				cornerLastBuf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}

			mBuf.unlock();                                        // 解锁缓冲队列

			TicToc t_whole;                                       // 整体处理时间计时器

			transformAssociateToMap();                            // 将里程计位姿变换到地图坐标系

			// === 动态调整地图立方体网格 ===
			TicToc t_shift;                                       // 网格调整时间计时器
			// 计算当前位置在网格中的索引
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;  // X轴网格索引
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight; // Y轴网格索引
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;  // Z轴网格索引

			// 处理负坐标情况
			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;

			// === 网格滑动窗口管理 ===
			// 当机器人移动超出当前网格范围时，需要滑动网格窗口
			while (centerCubeI < 3)                               // X轴负方向越界处理
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{ 
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI++;
				laserCloudCenWidth++;
			}

			while (centerCubeI >= laserCloudWidth - 3)
			{ 
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i < laserCloudWidth - 1; i++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			while (centerCubeJ < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			while (centerCubeK < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}

			int laserCloudValidNum = 0;
			int laserCloudSurroundNum = 0;

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{ 
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}

			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();


			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

			printf("map prepare time %f ms\n", t_shift.toc());
			printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
			{
				TicToc t_opt;
				TicToc t_tree;
				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
				printf("build tree time %f ms \n", t_tree.toc());

				// === Ceres优化器设置 ===
				for (int iterCount = 0; iterCount < 2; iterCount++)  // 进行2次迭代优化
				{
					//ceres::LossFunction *loss_function = NULL;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);    // Huber损失函数，减少异常值影响
					ceres::LocalParameterization *q_parameterization =
						new ceres::EigenQuaternionParameterization();                 // 四元数参数化
					ceres::Problem::Options problem_options;

					ceres::Problem problem(problem_options);                           // 创建优化问题
					problem.AddParameterBlock(parameters, 4, q_parameterization);      // 添加四元数参数块（4维）
					problem.AddParameterBlock(parameters + 4, 3);                      // 添加平移参数块（3维）

					TicToc t_data;                                                     // 数据关联时间计时器
					int corner_num = 0;                                                // 角点约束数量计数器

					// === 角点特征匹配与约束构建 ===
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						pointOri = laserCloudCornerStack->points[i];                   // 获取当前角点
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);                     // 将点变换到地图坐标系
						kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // 搜索5个最近邻

						if (pointSearchSqDis[4] < 1.0)                                 // 确保第5个邻居距离小于1米
						{ 
							std::vector<Eigen::Vector3d> nearCorners;
							Eigen::Vector3d center(0, 0, 0);                          // 5个最近邻的中心点
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;                                // 累积坐标
								nearCorners.push_back(tmp);                           // 存储邻居点
							}
							center = center / 5.0;                                    // 计算中心点

							// === 主成分分析（PCA）计算直线方向 ===
							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();          // 协方差矩阵
							for (int j = 0; j < 5; j++)
							{
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose(); // 构建协方差矩阵
							}

							// === 特征值分解确定直线性质 ===
							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat); // 特征值分解

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);   // 最大特征值对应的特征向量（直线方向）
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z); // 当前点
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])          // 确保是直线特征（最大特征值显著大于其他）
							{ 
								Eigen::Vector3d point_on_line = center;                    // 直线上的点
								Eigen::Vector3d point_a, point_b;                          // 直线上的两点
								point_a = 0.1 * unit_direction + point_on_line;            // 沿直线方向的点A
								point_b = -0.1 * unit_direction + point_on_line;           // 沿直线方向的点B

								// === 添加点到直线的约束 ===
								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								corner_num++;                                               // 角点约束计数器增加
							}							
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					int surf_num = 0;                                                   // 平面点约束数量计数器
					// === 平面特征匹配与约束构建 ===
					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];                      // 获取当前平面点
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);                      // 将点变换到地图坐标系
						kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); // 搜索5个最近邻

						Eigen::Matrix<double, 5, 3> matA0;                              // 用于拟合平面的矩阵
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
						if (pointSearchSqDis[4] < 1.0)                                  // 确保第5个邻居距离小于1米
						{
							
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x; // 填充邻居点坐标
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}
							// === 最小二乘法拟合平面方程 ax + by + cz + 1 = 0 ===
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0); // 求解平面法向量
							double negative_OA_dot_norm = 1 / norm.norm();                  // 归一化常数
							norm.normalize();                                               // 法向量归一化

							// Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;                                        // 平面有效性标志
							for (int j = 0; j < 5; j++)
							{
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
								{
									planeValid = false;                                     // 平面拟合质量检查
									break;
								}
							}
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z); // 当前点
							if (planeValid)                                                 // 平面有效时添加约束
							{
								// === 添加点到平面的约束 ===
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								surf_num++;                                                 // 平面点约束计数器增加
							}
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					//printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					//printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

					printf("mapping data assosiation time %f ms \n", t_data.toc()); // 数据关联时间

					// === Ceres求解器配置与优化 ===
					TicToc t_solver;                                                  // 求解器时间计时器
					ceres::Solver::Options options;                                  // 求解器选项
					options.linear_solver_type = ceres::DENSE_QR;                    // 使用稠密QR分解求解器
					options.max_num_iterations = 4;                                  // 最大迭代次数
					options.minimizer_progress_to_stdout = false;                    // 不输出进度信息
					options.check_gradients = false;                                 // 不检查梯度
					options.gradient_check_relative_precision = 1e-4;                // 梯度检查精度
					ceres::Solver::Summary summary;                                  // 求解器结果摘要
					ceres::Solve(options, &problem, &summary);                       // 执行优化求解
					printf("mapping solver time %f ms \n", t_solver.toc());         // 求解器时间

					//printf("time %f \n", timeLaserOdometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}
				printf("mapping optimization time %f \n", t_opt.toc());              // 优化总时间
			}
			else
			{
				ROS_WARN("time Map corner and surf num are not enough");             // 警告：特征点数量不足
			}
			transformUpdate();                                                        // 更新变换矩阵

			// === 添加新的特征点到地图 ===
			TicToc t_add;                                                            // 添加点云时间计时器
			// === 将角点添加到地图中 ===
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);   // 转换到地图坐标系

				// === 计算点所属的3D网格索引 ===
				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;    // X方向网格索引
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;   // Y方向网格索引  
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;    // Z方向网格索引

				// === 处理负坐标的情况 ===
				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				// === 检查网格索引是否在有效范围内 ===
				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK; // 计算线性索引
					laserCloudCornerArray[cubeInd]->push_back(pointSel);             // 将角点添加到对应网格
				}
			}

			// === 将平面点添加到地图中 ===
			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);     // 转换到地图坐标系

				// === 计算点所属的3D网格索引 ===
				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;    // X方向网格索引
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;   // Y方向网格索引
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;    // Z方向网格索引

				// === 处理负坐标的情况 ===
				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				// === 检查网格索引是否在有效范围内 ===
				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK; // 计算线性索引
					laserCloudSurfArray[cubeInd]->push_back(pointSel);               // 将平面点添加到对应网格
				}
			}
			printf("add points time %f ms\n", t_add.toc());                         // 添加点云时间

			
			// === 对有效网格内的点云进行降采样 ===
			TicToc t_filter;                                                        // 滤波时间计时器
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];                                    // 有效网格索引

				// === 角点降采样 ===
				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);     // 设置输入点云
				downSizeFilterCorner.filter(*tmpCorner);                            // 执行降采样
				laserCloudCornerArray[ind] = tmpCorner;                             // 更新角点数组

				// === 平面点降采样 ===
				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);         // 设置输入点云
				downSizeFilterSurf.filter(*tmpSurf);                               // 执行降采样
				laserCloudSurfArray[ind] = tmpSurf;                                 // 更新平面点数组
			}
			printf("filter time %f ms \n", t_filter.toc());                        // 滤波时间
			
			// === 发布点云地图 ===
			TicToc t_pub;                                                           // 发布时间计时器
			//publish surround map for every 5 frame
			if (frameCount % 5 == 0)                                                // 每5帧发布一次周围地图
			{
				laserCloudSurround->clear();                                        // 清空周围点云
				for (int i = 0; i < laserCloudSurroundNum; i++)
				{
					int ind = laserCloudSurroundInd[i];                             // 周围网格索引
					*laserCloudSurround += *laserCloudCornerArray[ind];             // 添加角点
					*laserCloudSurround += *laserCloudSurfArray[ind];               // 添加平面点
				}

				// === 发布周围地图点云 ===
				sensor_msgs::PointCloud2 laserCloudSurround3;
				pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);            // 转换为ROS消息
				laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry); // 设置时间戳
				laserCloudSurround3.header.frame_id = "/camera_init";               // 设置坐标系
				pubLaserCloudSurround.publish(laserCloudSurround3);                 // 发布消息
			}

			// === 每20帧发布完整地图 ===
			if (frameCount % 20 == 0)
			{
				pcl::PointCloud<PointType> laserCloudMap;                           // 完整地图点云
				for (int i = 0; i < 4851; i++)                                      // 遍历所有网格（21x21x11=4851）
				{
					laserCloudMap += *laserCloudCornerArray[i];                     // 添加角点
					laserCloudMap += *laserCloudSurfArray[i];                       // 添加平面点
				}
				// === 发布完整地图点云 ===
				sensor_msgs::PointCloud2 laserCloudMsg;
				pcl::toROSMsg(laserCloudMap, laserCloudMsg);                        // 转换为ROS消息
				laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry); // 设置时间戳
				laserCloudMsg.header.frame_id = "/camera_init";                     // 设置坐标系
				pubLaserCloudMap.publish(laserCloudMsg);                            // 发布完整地图
			}

			// === 发布全分辨率点云 ===
			int laserCloudFullResNum = laserCloudFullRes->points.size();            // 全分辨率点云数量
			for (int i = 0; i < laserCloudFullResNum; i++)
			{
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]); // 转换到地图坐标系
			}

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);                  // 转换为ROS消息
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry); // 设置时间戳
			laserCloudFullRes3.header.frame_id = "/camera_init";                    // 设置坐标系
			pubLaserCloudFullRes.publish(laserCloudFullRes3);                       // 发布全分辨率点云

			printf("mapping pub time %f ms \n", t_pub.toc());                      // 发布时间

			printf("whole mapping time %f ms +++++\n", t_whole.toc());             // 整个建图时间

			// === 发布里程计信息 ===
			nav_msgs::Odometry odomAftMapped;                                       // 建图后里程计
			odomAftMapped.header.frame_id = "/camera_init";                         // 父坐标系
			odomAftMapped.child_frame_id = "/aft_mapped";                           // 子坐标系
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);    // 时间戳
			// === 设置四元数姿态 ===
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
			// === 设置位置 ===
			odomAftMapped.pose.pose.position.x = t_w_curr.x();
			odomAftMapped.pose.pose.position.y = t_w_curr.y();
			odomAftMapped.pose.pose.position.z = t_w_curr.z();
			pubOdomAftMapped.publish(odomAftMapped);                                // 发布里程计

			// === 发布路径信息 ===
			geometry_msgs::PoseStamped laserAfterMappedPose;                        // 建图后位姿
			laserAfterMappedPose.header = odomAftMapped.header;                     // 复制头信息
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;                    // 复制位姿
			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;         // 设置路径时间戳
			laserAfterMappedPath.header.frame_id = "/camera_init";              // 设置路径坐标系
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);             // 添加位姿到路径
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);                  // 发布路径

			// === 发布TF变换 ===
			static tf::TransformBroadcaster br;                                     // TF广播器
			tf::Transform transform;                                                // 变换
			tf::Quaternion q;                                                       // 四元数
			// === 设置位置变换 ===
			transform.setOrigin(tf::Vector3(t_w_curr(0),
											t_w_curr(1),
											t_w_curr(2)));
			// === 设置旋转变换 ===
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());
			transform.setRotation(q);
			// === 广播TF变换 ===
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

			frameCount++;                                                           // 帧计数器增加
		}
		std::chrono::milliseconds dura(2);                                          // 休眠2毫秒
        std::this_thread::sleep_for(dura);                                          // 控制线程执行频率
	}
}

// === 主函数 ===
int main(int argc, char **argv)
{
	// === ROS初始化 ===
	ros::init(argc, argv, "laserMapping");                                          // 初始化ROS节点
	ros::NodeHandle nh;                                                             // 节点句柄

	// === 获取参数配置 ===
	float lineRes = 0;                                                              // 角点降采样分辨率
	float planeRes = 0;                                                             // 平面点降采样分辨率
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);                       // 从参数服务器获取角点分辨率，默认0.4
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);                     // 从参数服务器获取平面点分辨率，默认0.8
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);        // 打印分辨率参数
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);                     // 设置角点降采样体素大小
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);                   // 设置平面点降采样体素大小

	// === 订阅话题 ===
	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);     // 订阅角点点云

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);           // 订阅平面点点云

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);                              // 订阅激光里程计

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);                  // 订阅全分辨率点云

	// === 发布话题 ===
	pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);            // 发布周围地图点云

	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);                      // 发布完整地图点云

	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);        // 发布配准后的全分辨率点云

	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);                         // 发布建图后里程计

	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);       // 发布高频率里程计

	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);                         // 发布建图后路径

	// === 初始化点云数组 ===
	for (int i = 0; i < laserCloudNum; i++)                                         // 遍历所有网格（4851个）
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());           // 初始化角点点云数组
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());             // 初始化平面点点云数组
	}

	// === 启动建图处理线程 ===
	std::thread mapping_process{process};                                           // 创建建图处理线程

	// === 开始ROS事件循环 ===
	ros::spin();                                                                    // 进入ROS消息循环

	return 0;                                                                       // 程序结束
}
