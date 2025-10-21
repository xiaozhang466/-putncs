/*
 * ===================================================================
 * 【PUTN全局路径规划节点 - global_planning_node.cpp】
 * ===================================================================
 * 
 * 【功能描述】
 * 本文件是PUTN系统的全局路径规划核心节点，主要功能包括：
 * 1. 接收A-LOAM发布的点云地图并转换为体素网格
 * 2. 实现基于PF-RRT*算法的三维路径规划
 * 3. 提供可视化功能，在RViz中显示地图、路径和搜索树
 * 4. 为局部规划器提供全局路径指引
 * 
 * 【算法核心】
 * - PF-RRT*（Plane-Fitting RRT*）：适用于崎岖地形的路径规划算法
 * - 基于体素网格的碰撞检测和空间搜索
 * - 实时位姿更新和动态路径重规划
 * 
 * 【数据流】
 * A-LOAM点云地图 → 体素化 → PF-RRT*规划 → 全局路径 → 局部规划器
 * ===================================================================
 */

#include "backward.hpp"               // 错误处理和调试工具
#include "PUTN_planner.h"             // PUTN规划器核心类定义
#include <tf/transform_listener.h>    // TF坐标变换监听器
#include <visualization_msgs/Marker.h> // 可视化标记消息
#include <nav_msgs/Path.h>            // 路径消息类型
#include <std_msgs/Float32MultiArray.h> // 浮点数组消息

// 命名空间使用声明
using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

// ===================================================================
// 【错误处理和调试】
// ===================================================================
namespace backward
{
backward::SignalHandling sh;          // 信号处理，用于程序崩溃时的堆栈跟踪
}

// ===================================================================
// 【ROS通信相关】
// ===================================================================
// ROS订阅器
ros::Subscriber map_sub, wp_sub;      // 地图订阅器和路径点订阅器

// ROS发布器
ros::Publisher grid_map_vis_pub;      // 体素网格地图可视化发布器
ros::Publisher path_vis_pub;          // 路径可视化发布器  
ros::Publisher goal_vis_pub;          // 目标点可视化发布器
ros::Publisher surf_vis_pub;          // 表面点云可视化发布器
ros::Publisher tree_vis_pub;          // 搜索树可视化发布器
ros::Publisher path_interpolation_pub; // 路径插值结果发布器
ros::Publisher tree_tra_pub;          // 搜索树可通行性发布器

// ===================================================================
// 【全局状态变量】
// ===================================================================
// 机器人是否有移动目标的标志位
bool has_goal = false;

// 添加静态地图标志
bool use_static_map = false;
std::string pcd_file_path = "";

// ===================================================================
// 【规划参数配置】
// ===================================================================
// 从launch文件获取的仿真参数
double resolution;           // 体素网格分辨率（米）
double goal_thre;           // 目标阈值距离（米）
double step_size;           // RRT算法步长（米）
double h_surf_car;          // 车辆与表面高度差（米）
double max_initial_time;    // 最大初始化时间（毫秒）
double radius_fit_plane;    // 平面拟合半径（米）
FitPlaneArg fit_plane_arg;  // 平面拟合参数结构体
double neighbor_radius;     // 邻域搜索半径（米）

// ===================================================================
// 【核心全局变量】
// ===================================================================
Vector3d start_pt;          // 起始点坐标（机器人当前位置）
Vector3d target_pt;         // 目标点坐标（规划终点）
World* world = NULL;        // 世界环境指针（存储体素地图）
PFRRTStar* pf_rrt_star = NULL; // PF-RRT*规划器指针

// ===================================================================
// 【函数声明】
// ===================================================================
void rcvWaypointsCallback(const nav_msgs::Path& wp);                    // 接收路径点回调函数
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map); // 接收点云地图回调函数
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* _path_interpolation_pub); // 发布插值路径函数
void findSolution();                                                     // 寻找解决方案函数
void callPlanner();                                                      // 调用规划器函数

// ===================================================================
// 【回调函数实现】
// ===================================================================

/**
 * @brief 接收来自RViz的目标点
 * @param wp 包含目标点的路径消息
 * 
 * 功能说明：
 * - 从RViz的3D导航目标工具接收用户设定的目标点
 * - 提取目标点的三维坐标并存储到全局变量target_pt中
 * - 设置has_goal标志位，触发路径规划流程
 */
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
  if (!world->has_map_)    // 检查世界环境是否已经有地图数据
    return;
  has_goal = true;         // 设置目标标志位
  // 提取目标点的三维坐标（只使用第一个路径点作为目标）
  target_pt = Vector3d(wp.poses[0].pose.position.x, 
                      wp.poses[0].pose.position.y, 
                      wp.poses[0].pose.position.z);
  ROS_INFO("Receive the planning target");  // 输出接收到规划目标的信息
}

/**
 * @brief 接收点云数据构建体素网格地图
 * @param pointcloud_map 来自A-LOAM的点云地图消息
 * 
 * 功能说明：
 * - 接收A-LOAM发布的完整点云地图数据
 * - 将ROS点云消息转换为PCL点云格式
 * - 调用世界环境的初始化函数建立体素网格
 * - 遍历所有点云数据，将障碍物点标记到体素网格中
 * - 发布可视化信息供RViz显示
 */
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2& pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;    // 创建PCL点云对象
  pcl::fromROSMsg(pointcloud_map, cloud);  // 将ROS消息转换为PCL格式

  world->initGridMap(cloud);               // 初始化体素网格地图

  // 遍历点云中的每个点，将其标记为障碍物
  for (const auto& pt : cloud)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);   // 创建障碍物点的坐标向量
    world->setObs(obstacle);               // 在体素网格中标记该点为障碍物
  }
  visWorld(world, &grid_map_vis_pub);      // 发布可视化地图供RViz显示
}

/**
 * @brief 线性插值生成的路径以满足局部规划的需求
 * @param solution 规划算法生成的解决方案路径节点向量
 * @param path_interpolation_pub 路径插值发布器指针
 * 
 * 功能说明：
 * - 将PF-RRT*生成的离散路径节点进行线性插值
 * - 生成更密集的路径点以供局部规划器使用
 * - 发布插值后的路径数据（浮点数组格式）
 */
void pubInterpolatedPath(const vector<Node*>& solution, ros::Publisher* path_interpolation_pub)
{
  if (path_interpolation_pub == NULL)     // 检查发布器是否有效
    return;
  Float32MultiArray msg;                  // 创建浮点数组消息
  
  // 遍历解决方案中的每个节点
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)         // 处理最后一个节点（目标点）
    {
      // 添加最后一个节点的坐标到消息中
      msg.data.push_back(solution[i]->position_(0));  // X坐标
      msg.data.push_back(solution[i]->position_(1));  // Y坐标
      msg.data.push_back(solution[i]->position_(2));  // Z坐标
    }
    else
    {
      // 计算当前节点到下一节点之间需要插值的点数
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;  // 计算位置差向量
      
      // 在两个节点之间进行线性插值
      for (size_t j = 0; j < interpolation_num; j++)
      {
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        msg.data.push_back(interpt(0));    // 插值点X坐标
        msg.data.push_back(interpt(1));    // 插值点Y坐标
        msg.data.push_back(interpt(2));    // 插值点Z坐标
      }
    }
  }
  path_interpolation_pub->publish(msg);   // 发布插值后的路径消息
}

/**
 * @brief 在指定起点和目标的前提下，调用PF-RRT*算法进行路径规划
 * 
 * 功能说明：
 * - 根据起点和目标点的投影结果，分为三种情况进行处理
 * - 执行全局路径规划，生成可行路径
 * - 处理各种异常情况（起点无法投影、目标不可达等）
 * - 发布规划结果的可视化信息
 */
void findSolution()
{
  printf("=========================================================================\n");
  ROS_INFO("Start calling PF-RRT*");      // 开始调用PF-RRT*算法
  Path solution = Path();                 // 创建路径解决方案对象

  pf_rrt_star->initWithGoal(start_pt, target_pt);  // 使用起点和终点初始化PF-RRT*

  // 情况1: 当起点无法投影到表面时，PF-RRT*无法工作
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }
  // 情况2: 如果起点和目标都可以投影，PF-RRT*将执行全局规划并尝试生成路径
  else if (pf_rrt_star->state() == Global)
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;      // 最大迭代次数
    double max_time = 100.0;  // 最大规划时间（毫秒）

    // 在指定时间内尝试找到解决方案
    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);  // 执行规划算法
      max_time += 100.0;  // 增加时间限制并重试
    }

    if (!solution.nodes_.empty())  // 检查是否找到有效路径
      ROS_INFO("Get a global path!");
    else
      ROS_WARN("No solution found!");
  }
  // 情况3: 如果起点可以投影但目标不能，PF-RRT*将尝试寻找临时目标进行过渡
  else
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;      // 滚动规划模式的最大迭代次数
    double max_time = 100.0;  // 滚动规划模式的最大时间

    solution = pf_rrt_star->planner(max_iter, max_time);  // 执行滚动规划

    if (!solution.nodes_.empty())  // 检查是否找到子路径
      ROS_INFO("Get a sub path!");
    else
      ROS_WARN("No solution found!");
  }
  ROS_INFO("End calling PF-RRT*");    // 结束PF-RRT*调用
  printf("=========================================================================\n");

  // 发布规划结果的可视化信息
  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);  // 发布插值路径
  visPath(solution.nodes_, &path_vis_pub);                        // 可视化路径
  visSurf(solution.nodes_, &surf_vis_pub);                        // 可视化表面信息

  // 当PF-RRT*生成足够短的全局路径时，认为机器人已到达目标区域
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
  {
    has_goal = false;                               // 重置目标标志位
    visOriginAndGoal({}, &goal_vis_pub);           // 清除目标点可视化
    visPath({}, &path_vis_pub);                    // 清除路径可视化
    ROS_INFO("The Robot has achieved the goal!!!"); // 输出到达目标信息
  }

  // 如果没有找到路径解决方案，清除路径可视化
  if (solution.type_ == Path::Empty)
    visPath({}, &path_vis_pub);
}

/**
 * @brief 规划器调用控制函数
 * 
 * 功能说明：
 * - 根据是否有目标点来决定执行哪种规划模式
 * - 无目标时：执行空间探索，扩展搜索树
 * - 有目标时：调用findSolution()进行路径规划
 * - 控制初始化时间和树的大小
 */
void callPlanner()
{
  static double init_time_cost = 0.0;    // 静态变量记录初始化时间消耗
  if (!world->has_map_)                  // 检查是否有地图数据
    return;

  // 当没有目标且初始化时间小于1秒时，搜索树会以一定频率扩展以更充分地探索空间
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);          // 记录开始时间
    pf_rrt_star->initWithoutGoal(start_pt); // 无目标初始化
    timeval end;
    gettimeofday(&end, NULL);            // 记录结束时间
    // 计算初始化时间消耗（毫秒）
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    
    if (pf_rrt_star->state() == WithoutGoal)  // 检查是否成功进入无目标状态
    {
      int max_iter = 550;        // 无目标模式的最大迭代次数
      double max_time = 100.0;   // 无目标模式的最大时间
      pf_rrt_star->planner(max_iter, max_time);  // 执行无目标规划（空间探索）
      ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else
      ROS_WARN("The start point can't be projected,unable to execute PF-RRT* algorithm");
  }
  // 如果有指定的移动目标，调用PF-RRT*寻找解决方案
  else if (has_goal)
  {
    findSolution();           // 调用路径规划求解函数
    init_time_cost = 0.0;     // 重置初始化时间
  }
  // 初始化过程超过1秒后停止树的扩展
  else
    ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

// ===================================================================
// 【主函数】
// ===================================================================

/**
 * @brief 主函数 - 全局路径规划节点入口点
 * 
 * 功能说明：
 * - 初始化ROS节点和通信接口
 * - 配置规划参数和算法选项
 * - 创建世界环境和PF-RRT*规划器实例
 * - 运行主循环，实时更新机器人位置并执行规划
 */
int main(int argc, char** argv)
{
  // === ROS节点初始化 ===
  ros::init(argc, argv, "global_planning_node");  // 初始化ROS节点
  ros::NodeHandle nh("~");                         // 创建私有命名空间节点句柄

  // === ROS通信接口设置 ===
  // 订阅器设置
  map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);        // 订阅地图点云数据
  wp_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);    // 订阅路径点数据

  // 发布器设置  
  grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);       // 体素网格可视化
  path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 20);             // 路径可视化
  goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);              // 目标点可视化
  surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("surf_vis", 100);             // 表面可视化
  tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 1);              // 搜索树可视化
  tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>("tree_tra", 1);            // 树可通行性数据
  path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>("global_path", 1000); // 全局路径数据

  // === 参数配置从ROS参数服务器读取 ===
  // 地图参数
  nh.param("map/resolution", resolution, 0.1);               // 体素网格分辨率

  // 规划算法基本参数
  nh.param("planning/goal_thre", goal_thre, 1.0);            // 目标阈值距离
  nh.param("planning/step_size", step_size, 0.2);            // RRT算法步长
  nh.param("planning/h_surf_car", h_surf_car, 0.4);          // 车辆高度参数
  nh.param("planning/neighbor_radius", neighbor_radius, 1.0); // 邻域搜索半径

  // 平面拟合算法参数
  nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);        // 平面拟合总权重
  nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);   // 平坦度权重
  nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);            // 坡度权重
  nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);      // 稀疏性权重
  nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);       // 最小比率
  nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);        // 最大比率
  nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);     // 收敛阈值

  nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);         // 平面拟合半径
  nh.param("planning/max_initial_time", max_initial_time, 1000.0);      // 最大初始化时间

  // === 核心对象初始化 ===
  world = new World(resolution);                    // 创建世界环境对象
  pf_rrt_star = new PFRRTStar(h_surf_car, world);  // 创建PF-RRT*规划器对象

  // === PF-RRT*算法参数设置 ===
  pf_rrt_star->setGoalThre(goal_thre);                    // 设置目标阈值
  pf_rrt_star->setStepSize(step_size);                    // 设置步长
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);            // 设置平面拟合参数
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);      // 设置平面拟合半径
  pf_rrt_star->setNeighborRadius(neighbor_radius);       // 设置邻域半径

  // === 可视化发布器关联 ===
  pf_rrt_star->goal_vis_pub_ = &goal_vis_pub;    // 关联目标可视化发布器
  pf_rrt_star->tree_vis_pub_ = &tree_vis_pub;    // 关联搜索树可视化发布器
  pf_rrt_star->tree_tra_pub_ = &tree_tra_pub;    // 关联树可通行性发布器

  // === TF坐标变换监听器 ===
  tf::TransformListener listener;                // 创建TF监听器用于获取机器人位置

  // === 主循环 ===
  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);       // 记录循环开始时间

    // === 更新机器人当前位置 ===
    tf::StampedTransform transform;   // TF变换对象
    while (true && ros::ok())
    {
      try
      {
        // 查询从世界坐标系到机器人base_link的坐标变换
        listener.lookupTransform("/world", "/base_link", ros::Time(0), transform);
        break;                        // 成功获取变换，退出循环
      }
      catch (tf::TransformException& ex)
      {
        continue;                     // 获取失败，继续尝试
      }
    }
    // 提取机器人当前位置坐标并更新起始点
    start_pt << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    // === 执行ROS回调函数更新地图和检查新目标 ===
    ros::spinOnce();                  // 处理订阅消息的回调函数
    
    // === 调用PF-RRT*进行路径规划 ===
    callPlanner();                    // 执行规划器主逻辑
    
    // === 控制循环频率为100ms ===
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);       // 记录当前时间
      // 计算已消耗的时间（毫秒）
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 100);               // 确保每个循环至少100毫秒
  }
  return 0;                           // 程序正常结束
}
