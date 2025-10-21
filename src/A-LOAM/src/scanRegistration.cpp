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


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
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

using std::atan2;
using std::cos;
using std::sin;

// 激光雷达扫描周期，单位：秒（10Hz = 0.1s）
const double scanPeriod = 0.1;

// 系统初始化相关变量
const int systemDelay = 0;         // 系统延迟帧数，0表示不延迟
int systemInitCount = 0;           // 初始化计数器
bool systemInited = false;         // 系统是否已初始化标志

// 激光雷达线数（16、32或64线）
int N_SCANS = 0;

// 全局数组，存储点云处理相关信息（最大支持40万个点）
float cloudCurvature[400000];      // 存储每个点的曲率值
int cloudSortInd[400000];          // 存储点的索引，用于按曲率排序
int cloudNeighborPicked[400000];   // 标记点是否已被选为特征点（1=已选，0=未选）
int cloudLabel[400000];            // 点的标签：2=强角点，1=弱角点，-1=平面点，0=普通点

// 比较函数：按曲率从小到大排序
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

// ROS发布器：发布不同类型的点云数据
ros::Publisher pubLaserCloud;           // 发布处理后的完整点云
ros::Publisher pubCornerPointsSharp;    // 发布强角点（边缘特征点）
ros::Publisher pubCornerPointsLessSharp;// 发布弱角点 
ros::Publisher pubSurfPointsFlat;       // 发布强平面点
ros::Publisher pubSurfPointsLessFlat;   // 发布弱平面点（经过下采样）
ros::Publisher pubRemovePoints;         // 发布被移除的点
std::vector<ros::Publisher> pubEachScan;// 发布每条扫描线的点云（可选）

// 是否发布每条扫描线的点云（用于调试）
bool PUB_EACH_LINE = false;

// 激光雷达最小有效距离，小于此距离的点将被过滤掉
double MINIMUM_RANGE = 0.1; 

/**
 * @brief 移除距离传感器过近的点云
 * 
 * 激光雷达在近距离处容易产生噪声，需要过滤掉这些点
 * 
 * @param cloud_in 输入点云
 * @param cloud_out 输出点云
 * @param thres 距离阈值，小于此距离的点将被移除
 */
template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // 如果输入输出不是同一个对象，则复制头信息和调整大小
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;  // 输出点云的有效点计数器

    // 遍历所有输入点
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        // 计算点到原点的距离的平方：x²+y²+z²
        // 如果距离小于阈值，跳过这个点（不添加到输出点云中）
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];  // 保留有效点
        j++;
    }
    
    // 如果有点被过滤掉，调整输出点云大小
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    // 设置点云属性
    cloud_out.height = 1;                          // 无序点云的高度为1
    cloud_out.width = static_cast<uint32_t>(j);    // 宽度为有效点数
    cloud_out.is_dense = true;                     // 标记为稠密点云（无NaN点）
}

/**
 * @brief 激光点云处理的主要回调函数
 * 
 * 这个函数是整个特征提取过程的核心，主要步骤：
 * 1. 将ROS消息转换为PCL格式
 * 2. 按扫描线组织点云数据
 * 3. 计算每个点的曲率
 * 4. 根据曲率提取特征点
 * 5. 发布4种类型的特征点云
 * 
 * @param laserCloudMsg 输入的激光点云ROS消息
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    // === 第一步：系统初始化检查 ===
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;  // 如果还没初始化完成，直接返回
    }

    // === 第二步：数据准备和格式转换 ===
    TicToc t_whole;                                    // 计时：整个处理过程
    TicToc t_prepare;                                  // 计时：数据准备阶段
    std::vector<int> scanStartInd(N_SCANS, 0);        // 记录每条扫描线的起始索引
    std::vector<int> scanEndInd(N_SCANS, 0);          // 记录每条扫描线的结束索引

    // 将ROS的PointCloud2消息转换为PCL的PointXYZ格式
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // 数据清理：移除NaN点和过近的点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);  // 移除无效点
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);  // 移除过近点

    // === 第三步：计算扫描角度范围 ===
    int cloudSize = laserCloudIn.points.size();
    // 计算第一个点的角度（起始角度）
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    // 计算最后一个点的角度（结束角度），加2π确保角度范围正确
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    // 确保角度差在合理范围内（π到3π之间）
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    // === 第四步：按扫描线分组并添加时间戳信息 ===
    bool halfPassed = false;                           // 标记是否已经扫描了一半
    int count = cloudSize;                             // 有效点计数
    PointType point;                                   // 临时点变量
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);  // 按扫描线分组的点云

    // 处理每个点
    for (int i = 0; i < cloudSize; i++)
    {
        // 复制坐标信息
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // === 计算点属于哪条扫描线 ===
        // 通过垂直角度确定扫描线ID
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        // 根据不同型号的Velodyne激光雷达进行角度映射
        if (N_SCANS == 16)          // VLP-16: 垂直角度范围 -15°到+15°
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;            // 超出范围的点不处理
                continue;
            }
        }
        else if (N_SCANS == 32)     // HDL-32E: 垂直角度范围约 -30.67°到+10.67°
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)     // HDL-64E: 垂直角度范围 -24.33°到+2°
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // 使用[0 50]范围，>50的作为异常值移除
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        // === 计算点的时间戳信息 ===
        // 计算当前点的水平角度
        float ori = -atan2(point.y, point.x);
        
        // 处理角度的连续性问题（避免跨越±π时的跳跃）
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            // 如果已经扫描超过半圈，设置标志位
            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        // 计算相对时间（0到1之间的值）
        float relTime = (ori - startOri) / (endOri - startOri);
        
        // 将扫描线ID和时间信息编码到intensity字段
        // intensity = scanID + scanPeriod * relTime
        // 例如：5.023表示第5条扫描线，相对时间0.023
        point.intensity = scanID + scanPeriod * relTime;
        
        // 将点添加到对应的扫描线点云中
        laserCloudScans[scanID].push_back(point); 
    }
    // === 第五步：重新组织点云数据 ===
    cloudSize = count;                                 // 更新有效点数
    printf("points size %d \n", cloudSize);

    // 将分散在各扫描线中的点云合并成一个完整的点云
    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { 
        // 记录每条扫描线在合并点云中的起始和结束位置
        // +5和-6是为了曲率计算时避免边界问题
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];             // 合并点云
        scanEndInd[i] = laserCloud->size() - 6;
    }

    printf("prepare time %f \n", t_prepare.toc());

    // === 第六步：计算每个点的曲率 ===
    // 使用11点邻域（前5个+当前+后5个）计算曲率
    for (int i = 5; i < cloudSize - 5; i++)
    { 
        // X方向的二阶差分：前5个点的和 + 后5个点的和 - 10倍当前点
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        // Y方向的二阶差分
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        // Z方向的二阶差分
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        // 曲率 = 三个方向差分的平方和
        // 曲率大表示该点在边缘或角落（适合做角点特征）
        // 曲率小表示该点在平面上（适合做平面特征）
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;                           // 初始化排序索引
        cloudNeighborPicked[i] = 0;                    // 初始化为未选择
        cloudLabel[i] = 0;                             // 初始化标签为普通点
    }

    // === 第七步：特征点提取 ===
    TicToc t_pts;                                      // 计时：特征点提取

    // 创建4种类型的特征点云
    pcl::PointCloud<PointType> cornerPointsSharp;     // 强角点：曲率最大的点
    pcl::PointCloud<PointType> cornerPointsLessSharp; // 弱角点：曲率较大的点
    pcl::PointCloud<PointType> surfPointsFlat;        // 强平面点：曲率最小的点  
    pcl::PointCloud<PointType> surfPointsLessFlat;    // 弱平面点：其他点经过下采样

    float t_q_sort = 0;                                // 排序时间统计
    
    // 对每条扫描线进行特征提取
    for (int i = 0; i < N_SCANS; i++)
    {
        // 如果该扫描线点数太少，跳过
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
            
        // 用于存储当前扫描线的弱平面点（下采样前）
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        
        // 将每条扫描线分成6个子区域，分别提取特征
        // 这样做可以确保特征点在扫描线上分布均匀
        for (int j = 0; j < 6; j++)
        {
            // 计算当前子区域的起始和结束索引
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // 对当前子区域的点按曲率从小到大排序
            TicToc t_tmp;
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            // === 提取角点特征（曲率大的点） ===
            int largestPickedNum = 0;
            // 从曲率最大的点开始选择（ep到sp，逆序遍历）
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];                 // 当前曲率排序后的点索引

                // 检查该点是否满足角点条件：
                // 1. 没有被选为其他特征点的邻居
                // 2. 曲率大于0.1（经验阈值）
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {
                    largestPickedNum++;
                    if (largestPickedNum <= 2)             // 选择曲率最大的前2个点作为强角点
                    {                        
                        cloudLabel[ind] = 2;               // 标记为强角点
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]); // 强角点同时也是弱角点
                    }
                    else if (largestPickedNum <= 20)       // 选择前20个点作为弱角点
                    {                        
                        cloudLabel[ind] = 1;               // 标记为弱角点
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;                             // 超过20个点就停止选择
                    }

                    // 将当前点标记为已选择，避免重复选择
                    cloudNeighborPicked[ind] = 1; 

                    // === 邻居点抑制策略 ===
                    // 为了避免特征点过于密集，将选中点附近的点也标记为已选择
                    // 向后检查5个邻居点
                    for (int l = 1; l <= 5; l++)
                    {
                        // 计算相邻两点的距离
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        // 如果相邻点距离过大（>0.05m），说明不连续，停止抑制
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1; // 标记邻居点为已选择
                    }
                    // 向前检查5个邻居点
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // === 提取平面特征（曲率小的点） ===
            int smallestPickedNum = 0;
            // 从曲率最小的点开始选择（sp到ep，正序遍历）
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                // 检查该点是否满足平面点条件：
                // 1. 没有被选为其他特征点的邻居
                // 2. 曲率小于0.1（经验阈值）
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1;                  // 标记为平面点
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)            // 每个子区域最多选择4个平面点
                    { 
                        break;
                    }

                    // 同样进行邻居点抑制
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            // === 收集弱平面点 ===
            // 将所有未被选为强特征的点作为弱平面点的候选
            for (int k = sp; k <= ep; k++)
            {
                // 只要不是强角点(2)和弱角点(1)，都可以作为弱平面点
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // === 对弱平面点进行体素网格下采样 ===
        // 弱平面点数量通常很多，需要下采样来减少计算量
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);        // 体素大小：20cm × 20cm × 20cm
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        // 将下采样后的弱平面点添加到总的弱平面点云中
        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());


    // === 第八步：发布处理结果 ===
    // 将PCL格式的点云转换为ROS消息格式并发布

    // 发布处理后的完整点云（包含时间戳信息）
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    // 发布强角点特征
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    // 发布弱角点特征
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    // 发布强平面点特征
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    // 发布弱平面点特征（经过下采样）
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // === 可选：发布每条扫描线的点云（用于调试和可视化） ===
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "/camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    // === 性能统计和警告 ===
    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");  // 如果处理时间超过100ms，发出警告
}

/**
 * @brief 主函数 - 程序入口点
 * 
 * 初始化ROS节点，设置参数，创建订阅者和发布者
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    // 从ROS参数服务器读取激光雷达线数，默认为16线
    nh.param<int>("scan_line", N_SCANS, 16);

    // 从ROS参数服务器读取最小距离阈值，默认为0.1米
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    // 检查激光雷达线数是否支持
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // === 创建订阅者 ===
    // 订阅原始激光点云数据
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    // === 创建发布者 ===
    // 发布处理后的完整点云
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    // 发布4种类型的特征点云
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);          // 强角点

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);  // 弱角点

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);              // 强平面点

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);     // 弱平面点

    // 发布被移除的点（当前未使用）
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    // === 可选：为每条扫描线创建发布者（用于调试） ===
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    
    // 进入ROS消息循环，等待并处理激光点云数据
    ros::spin();

    return 0;
}
