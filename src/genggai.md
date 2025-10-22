2024-10-21
- （1）梳理PUTN在松灵RANGER MINI 2.0 + LSLiDAR上的适配清单，明确雷达话题与帧名称调整、A-LOAM参数选择、底盘驱动接口以及PUTN规划参数待调项，当前仅做分析未触及源码。

- （2）调整`putn_launch/launch/bringup.launch`雷达部分：集成LSLiDAR C16驱动/解码节点并统一点云话题为`/velodyne_points`，新增底盘到雷达的静态TF参数，且切换A-LOAM为VLP-16配置以匹配16线雷达。

- （3）根据车辆实际安装位置，将静态TF默认平移更新为前向0.15 m、竖直0.7 m（底盘0.3 m+雷达1.0 m）以匹配激光雷达相对`base_link`的姿态。

- （4）在`putn_launch/launch/bringup.launch`中增加松灵Ranger Mini V2底盘驱动参数并包含`ranger_bringup/ranger_mini_v2.launch`，统一CAN口/里程帧配置，启动底盘节点与PUTN帧体系对齐。

- （5）新增`scripts/setup_can0.sh`，封装说明书中的 CAN 初始化流程（加载 gs_usb、设置 can0、检查接口并提示安装 can-utils），上线前运行可快速完成底盘通信准备。

- （6）默认关闭底盘节点的`publish_odom_tf`，避免与A-LOAM `/aft_mapped→/base_link` TF 冲突；如需底盘自身发布 TF，可在launch中覆盖为 true。

- （7）优化A-LOAM时间戳同步逻辑：`laserOdometry.cpp`、`laserMapping.cpp`增加1ms容差并在检测到不同步时丢弃最旧数据替代直接`ROS_BREAK()`，避免实机无GPS时因轻微延迟导致节点退出。
