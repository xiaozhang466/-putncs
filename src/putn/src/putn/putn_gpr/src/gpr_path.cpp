/**
 * @file gpr_path.cpp
 * @brief 基于高斯过程回归的路径预测节点
 * @description 该节点接收全局规划的树结构和路径数据，使用高斯过程回归预测地形高度
 */

#include "gaussian_process_regression/gaussian_process_regression.h"  // 高斯过程回归库头文件
#include <ros/ros.h>                                                // ROS核心头文件
#include <fstream>                                                  // 文件操作头文件
#include <std_msgs/Float32MultiArray.h>                            // ROS Float32数组消息类型

using namespace std;    // 使用标准命名空间
using namespace Eigen;  // 使用Eigen矩阵库命名空间

// ============ 全局变量定义 ============
// 高斯过程回归超参数
double length_scale ;  // 长度尺度参数，控制输入距离对输出的影响
double sigma_f ;       // 信号方差，控制函数值的变化幅度
double sigma_n ;       // 噪声方差，表示观测噪声水平

// 输入输出维度定义
const size_t input_dim(2), output_dim(1);  // 输入维度2(x,y)，输出维度1(z高度)

// 数据类型定义
typedef Eigen::Matrix<float,3,1> input_type;   // 输入类型：3D向量(x,y,z)
typedef Eigen::Matrix<float,1,1> output_type;  // 输出类型：1D向量(高度值)

// 训练和测试数据容器
std::vector<input_type> train_inputs, test_inputs;     // 输入数据向量
std::vector<output_type> train_outputs, test_outputs;  // 输出数据向量

// ROS发布器，用于发布预测结果
ros::Publisher _surf_predict_pub;

// 超参数配置文件路径
string filepath;
/**
 * @brief 从文件加载训练数据
 * @param fname 文件名
 * @param inputs 输入数据容器
 * @param outputs 输出数据容器
 * @param input_dim 输入数据维度
 * @param output_dim 输出数据维度
 * @description 从指定文件读取训练数据，每行包含输入和输出数据
 */
template<typename input_type, typename output_type>
void load_data(const char *fname, std::vector<input_type> &inputs, std::vector<output_type> &outputs, int input_dim, int output_dim) {
  std::cout<<"entry this branch........"<<std::endl;
  
  // 声明临时变量用于存储读取的数据
  input_type inp,tinp;
  output_type outp,toutp;
  
  // 打开文件
  std::ifstream myfile(fname);
  if (!myfile.is_open())
  {
      std::cout << "Fail to open the file" << std::endl;
      return;
  }
  
  // 逐行读取文件内容
  std::string line;
  while(getline(myfile,line)){
      std::cout<<line<<" ";
    std::istringstream line_stream(line);
    
    // 读取输入数据的每个维度
    for(size_t k = 0; k < input_dim; k++)
      line_stream>>inp(k);
    
    // 读取输出数据的每个维度  
    for(size_t k = 0; k < output_dim; k++)
      line_stream>>outp(k);
    
    // 将读取的数据添加到容器中
    inputs.push_back(inp);
    outputs.push_back(outp);
  }
  std::cout<<"finish loading..."<<std::endl;
}
/**
 * @brief 从文件设置高斯过程回归的超参数
 * @param fname 超参数配置文件名
 * @param gpr 高斯过程回归对象引用
 * @description 从配置文件读取长度尺度、信号方差和噪声方差参数
 */
template<typename R>
void set_hyperparameters_from_file(const char *fname, GaussianProcessRegression<R> & gpr) {
  std::ifstream myfile;
  myfile.open(fname);
  if (!myfile.is_open())
  {
      std::cout << "Fail to open the file" << std::endl;
      return;
  }
  
  // 读取三个超参数：l(长度尺度), f(信号方差), n(噪声方差)
  R l, f, n;
  myfile>>l>>f>>n;
  myfile.close();
  
  // 设置高斯过程回归的超参数
  gpr.SetHyperParams(l,f,n);
}

/**
 * @brief 清空输入数据向量
 * @param vt 需要清空的输入数据向量引用
 * @description 使用swap技术快速释放vector内存，避免内存泄漏
 */
void ClearVectorIn( vector< input_type >& vt ) 
{
  vector< input_type >  veTemp; 
  veTemp.swap( vt );
}

/**
 * @brief 清空输出数据向量
 * @param vt 需要清空的输出数据向量引用
 * @description 使用swap技术快速释放vector内存，避免内存泄漏
 */
void ClearVectorOut( vector< output_type >& vt ) 
{
  vector< output_type > veTemp; 
  veTemp.swap( vt );
}

/**
 * @brief 树结构数据回调函数
 * @param msg ROS Float32MultiArray消息指针，包含树结构的训练数据
 * @description 接收全局规划节点发布的树结构数据，进行高斯过程回归训练和预测
 *              数据格式：每4个元素为一组(x, y, z, height)，其中(x,y,z)为输入，height为输出
 */
void treecb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // 性能计时变量
  double dur;
  clock_t start,end;start = clock();
  
  ROS_INFO("[node] receive the tree");
  
  // 检查消息是否为空
  if(msg->data.size() == 0) return;

  // 计算数据点数量（每4个元素为一组）
  int num = (int)(msg->data.size()/4);
  
  // 解析训练数据
	for (int i=0; i<num; i++) 
  {
    // 构建输入向量(x, y, z)
    input_type tmp_in;
    tmp_in << msg->data[4*i],msg->data[4*i+1],msg->data[4*i+2];
    train_inputs.push_back(tmp_in);
    
    // 构建输出向量(height)
    output_type tmp_out;
    tmp_out << msg->data[4*i+3] ;
    train_outputs.push_back(tmp_out);
	}

  // 创建高斯过程回归对象
  GaussianProcessRegression<float> myGPR(input_dim, output_dim);
  
  // 从文件设置超参数
  set_hyperparameters_from_file(filepath.c_str(),myGPR);

  // 添加训练数据到GPR模型
  for(size_t k=0; k<train_inputs.size(); k++){
      myGPR.AddTrainingData(train_inputs[k], train_outputs[k]);
   }
  
  // 阈值参数（当前未使用）
  double threshold = 0.1;

  // 如果没有测试数据，清空所有向量并返回
  if(test_inputs.size()==0)
  {
    ClearVectorIn(train_inputs);
    ClearVectorIn(test_inputs);
    ClearVectorOut(train_outputs);
    ClearVectorOut(test_outputs);
  }

  // 准备输出消息
   std_msgs::Float32MultiArray out_ym;   // 预测结果消息
   std_msgs::Float32MultiArray out_ys;   // 方差结果消息（当前未使用）

  // 对测试数据进行预测
  for(size_t k=0; k<test_inputs.size(); k++)
  {
    // 执行高斯过程回归预测
    auto outp = myGPR.DoRegression(test_inputs[k]);
    output_type tmp_out;
    tmp_out <<outp;
    test_outputs.push_back(tmp_out);
    
    // 计算预测协方差（不确定性）
    auto outp_cov = myGPR.DoRegressioncov(test_inputs[k]);
    
    // 将预测结果打包：[x, y, z, predicted_height, covariance]
    out_ym.data.push_back(test_inputs[k](0,0));  // x坐标
    out_ym.data.push_back(test_inputs[k](1,0));  // y坐标
    out_ym.data.push_back(test_inputs[k](2,0));  // z坐标
    out_ym.data.push_back(tmp_out(0,0));         // 预测高度
    out_ym.data.push_back(outp_cov(0,0));        // 预测方差

  }
  
  // 发布预测结果
  _surf_predict_pub.publish(out_ym);
  
  // 清理临时变量
  std_msgs::Float32MultiArray tmp_out;
  
  // 清空所有数据向量，释放内存
  ClearVectorIn(train_inputs);
  ClearVectorIn(test_inputs);
  ClearVectorOut(train_outputs);
  ClearVectorOut(test_outputs);
  
  // 计算并输出处理时间
  end = clock();
  dur = (double)(end - start);
  cout<<"Time consume ："<<dur/1000<<" ms"<<endl;
}

/**
 * @brief 路径数据回调函数
 * @param msg ROS Float32MultiArray消息指针，包含全局路径的测试数据
 * @description 接收全局规划节点发布的路径数据，作为高斯过程回归的测试输入
 *              数据格式：每3个元素为一组(x, y, z)，表示路径点的3D坐标
 */
void pathcb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  ROS_INFO("[node] receive the path");
  
  // 检查消息是否为空
  if(msg->data.size() == 0) return;

  // 计算路径点数量（每3个元素为一组）
  int num = (int)(msg->data.size()/3) ;
  
  // 解析路径点数据
	for (int i=0; i<num; i++) 
  {
    // 构建测试输入向量(x, y, z)
    input_type tmp_in;
    tmp_in <<msg->data[3*i],msg->data[3*i+1],msg->data[3*i+2];
    test_inputs.push_back(tmp_in);
	}
}

/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 * @description 初始化ROS节点，设置订阅者和发布者，进入事件循环
 */
int main(int argc, char **argv)
{
    // 初始化ROS节点，节点名为"GPR"
    ros::init (argc, argv, "GPR");
    
    // 创建私有节点句柄
    ros::NodeHandle ph("~");
    
    // 声明订阅者变量
    ros::Subscriber _tree_sub,_path_sub;
    
    // 订阅树结构数据话题，队列大小为1，回调函数为treecb
    _tree_sub = ph.subscribe( "/global_planning_node/tree_tra",  1,treecb  );
    
    // 订阅全局路径数据话题，队列大小为1，回调函数为pathcb
    _path_sub = ph.subscribe( "/global_planning_node/global_path",  1,pathcb  );
    
    // 创建预测结果发布者，话题名为"/surf_predict_pub"，队列大小为1
    _surf_predict_pub = ph.advertise< std_msgs::Float32MultiArray>("/surf_predict_pub",1);

    // 从ROS参数服务器获取配置文件路径
    ph.param<string>("file/cfg_path",filepath,"");   
    
    // 进入ROS事件循环，持续处理回调函数
    while(ros::ok())
    {
        ros::spinOnce();  // 处理一次回调函数
    }
    
    return 0;  // 程序正常退出
};
