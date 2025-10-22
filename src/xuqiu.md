10月21日：
    （1）我想在实机上测试putn这个开源项目的效果，putn的仓库我已经导入进来了，分别是A-LOAM和putn算法。我现在使用的小车底盘是松灵RANGER MINI 2.0，我已经把它的两个驱动仓库拉取到当前的项目路径下了，分别是ranger_ros和ugv_sdk这两个文件夹。雷达的相关代码我也已经导入进来了，在lslidar文件夹下。我还没有开始对代码适配的，请你帮我看看要做出什么适配？

    （2）先做第一个适配：现将雷达部分改好，项目的总启动文件是bringup.launch，现在实车上的雷达就是16线的，继续把bringup.lanuch文件当做启动文件进行适配

    （3）车底盘据地面大概30cm，激光雷达平放，据地面1米。激光雷达不在正中心，在中心前方15cm左右。

    （4）接下来做松灵底盘的适配

    （5）松灵底盘的用户说明书上是这样启动的：
        sudo modprobe gs_usb
        sudo ip link set can0 up type can bitrate 500000
        ifconfig -a            # 检查 can0 是否存在
        sudo apt install -y can-utils
        candump can0           # 监听底盘数据，验证连通性