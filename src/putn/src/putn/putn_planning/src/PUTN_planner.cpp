/**
 * @file PUTN_planner.cpp
 * @brief PUTN路径规划器实现文件
 * @description 基于RRT*算法的路径规划器，支持地形感知和平面拟合的3D路径规划
 * @author PUTN项目组
 */

#include "PUTN_planner.h"
#include <std_msgs/Float32MultiArray.h>  // ROS Float32数组消息类型
#include <random>                        // 随机数生成库

// 使用标准命名空间
using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace ros;
using namespace PUTN;
using namespace PUTN::visualization;
using namespace PUTN::planner;

/**
 * @brief 默认构造函数
 */
PFRRTStar::PFRRTStar(){}

/**
 * @brief 带参数构造函数
 * @param height 机器人离地高度
 * @param world 世界环境指针
 */
PFRRTStar::PFRRTStar(const double &height,World* world):h_surf_(height),world_(world){}

/**
 * @brief 析构函数
 * @description 清理树结构内存并删除目标节点
 */
PFRRTStar::~PFRRTStar()
{
    clean_vector(tree_);      // 清理树结构
    delete node_target_;      // 删除目标节点
    node_target_=NULL;        // 置空指针
}

/**
 * @brief 使用目标点初始化规划器
 * @param start_pos 起始位置（3D坐标）
 * @param end_pos 目标位置（3D坐标）  
 * @description 初始化规划器的起点和终点，并根据当前的规划状态决定是否继承之前的路径或树结构
 *              支持三种规划状态：Global（全局规划）、Roll（滚动规划）、WithoutGoal（无目标规划）
 */
// initWithGoal() 会初始化规划器的起点和终点，并根据当前的规划状态决定是否继承之前的路径或树结构。
void PFRRTStar::initWithGoal(const Vector3d &start_pos,const Vector3d &end_pos)
{
    Vector2d last_end_pos_2D=end_pos_2D_;           // 记录上一次的终点位置（2D投影）
    PlanningState last_planning_state=planning_state_; // 记录上一次的规划状态
    
    // 重置迭代次数和时间
    curr_iter_=0;
    curr_time_=0.0;
    
    // 将3D终点投影到2D平面
    end_pos_2D_=project2plane(end_pos);

    // 在起始位置拟合平面并创建节点
    Node* node_origin=fitPlane(start_pos);

    // 检查起始节点是否有效
    if(node_origin==NULL)
    {
        planning_state_=Invalid;  // 设置为无效状态
        return;
    }
    
    // 在目标位置拟合平面并创建节点
    Node* node_target=fitPlane(end_pos);

    // 根据目标节点是否有效确定规划状态
    // 如果目标节点无效，则使用Roll模式（滚动规划）；否则使用Global模式（全局规划）
    planning_state_=(node_target==NULL)?Roll:Global;

    // 清空闭环检查记录
    close_check_record_.clear();

    // 是否继承之前的规划结果
    bool inherit_flag=false;

    // 根据当前规划状态处理继承逻辑
    switch(planning_state_)
    {
        case Global:  // 全局规划模式
        {
            switch(last_planning_state)
            {
                case Global:
                    // 如果终点位置相同且可以继承全局路径，则继承路径
                    if(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Global))
                    {
                        inherit_flag=true;
                        delete node_target;node_target=NULL;  // 删除临时目标节点
                    }
                    else{delete node_target_;node_target_=node_target;}  // 更新目标节点
                    break;
                case WithoutGoal:
                    // 从无目标状态转换到有目标状态，尝试继承树结构
                    delete node_target_;node_target_=node_target;
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    // 其他状态直接更新目标节点
                    delete node_target_;node_target_=node_target;
                    break;
            }
        }
        break;
        case Roll:  // 滚动规划模式
        {
            switch(last_planning_state)
            {
                case Roll:
                    // 如果终点位置相同且可以继承子路径，则继承路径
                    inherit_flag=(last_end_pos_2D==end_pos_2D_ && inheritPath(node_origin,Path::Sub));
                    break;
                case WithoutGoal:
                    // 从无目标状态转换，尝试继承树结构
                    inherit_flag=inheritTree(node_origin);
                    break;
                default:
                    break;
            }
        }
        break;
        default:
        break;
    }

    // 如果无法继承之前的规划结果，则重新初始化
    if(!inherit_flag)
    {
        path_=Path();                    // 清空路径
        clean_vector(tree_);             // 清空树结构
        node_origin_=node_origin;        // 设置新的起始节点
        tree_.push_back(node_origin_);   // 将起始节点添加到树中
    }

    // 如果是Roll模式且终点位置发生变化，重置子目标阈值
    if(planning_state_==Roll && last_end_pos_2D!=end_pos_2D_) 
        sub_goal_threshold_=1.0f;

    // 准备可视化数据：起始节点和目标节点
    vector<Node*> origin_and_goal{node_origin_};
    if(planning_state_==Global) origin_and_goal.push_back(node_target_);
    
    // 发布起始点和目标点的可视化信息
    visOriginAndGoal(origin_and_goal,goal_vis_pub_);
}

/**
 * @brief 无目标初始化规划器
 * @param start_pos 起始位置（3D坐标）
 * @description 初始化规划器的起点，用于探索性规划（无明确目标的自主探索）
 */
// initWithoutGoal() 会初始化规划器的起点，并根据当前的规划状态决定是否继承之前的树结构。
void PFRRTStar::initWithoutGoal(const Vector3d &start_pos)
{
    // 重置迭代次数和时间
    curr_iter_=0;
    curr_time_=0.0;

    // 清空相关记录
    close_check_record_.clear();  // 清空闭环检查记录
    path_=Path();                 // 清空当前路径

    // 记录上一次的规划状态
    PlanningState last_planning_state=planning_state_;
  
    // 在起始位置拟合平面并创建节点
    Node* node_origin=fitPlane(start_pos);

    // 检查起始节点是否有效
    if(node_origin==NULL)
    {
        planning_state_=Invalid;  // 设置为无效状态
        return;
    }

    // 设置为无目标规划状态
    planning_state_=WithoutGoal;

    // 删除目标节点（无目标模式不需要目标节点）
    delete node_target_;
    node_target_=NULL;

    // 如果上一次不是无目标状态或无法继承树结构，则重新初始化
    if(last_planning_state!=WithoutGoal || !inheritTree(node_origin))
    {
        clean_vector(tree_);             // 清空树结构
        node_origin_=node_origin;        // 设置新的起始节点
        tree_.push_back(node_origin_);   // 将起始节点添加到树中
    }
}

/**
 * @brief 递归更新节点成本
 * @param node_input 要更新的节点指针
 * @description 递归地更新输入节点及其子节点的成本，并执行闭环检查
 *              成本计算基于父节点成本加上到父节点的距离代价
 */
// updateNode() 会递归地更新输入节点及其子节点的成本，并检查它们是否在闭环检查记录中。
void PFRRTStar::updateNode(Node* node_input)
{
    // 跳过根节点，计算其他节点的成本
    if(node_input->parent_!=NULL)
        node_input->cost_=node_input->parent_->cost_+calCostBetweenTwoNode(node_input,node_input->parent_);

    // 检查节点是否接近目标（闭环检查）
    closeCheck(node_input);

    // 递归更新所有子节点
    for(auto &node:node_input->children_) updateNode(node);
}

/**
 * @brief 递归添加无效节点到列表
 * @param node_input 要检查的节点指针（引用）
 * @param ifdelete 是否强制删除标志
 * @param invalid_nodes 无效节点列表（输出参数）
 * @description 递归地检查输入节点及其子节点是否有效，将无效节点添加到列表中
 *              无效条件：1)平面拟合失败 2)与父节点间存在碰撞 3)起始位置不可通行
 */
// addInvalidNodes() 会递归地检查输入节点及其子节点是否有效，并将无效的节点添加到 invalid_nodes 向量中。
void PFRRTStar::addInvalidNodes(Node* &node_input,const bool &ifdelete,vector<Node*> &invalid_nodes)
{
    if(node_input==NULL) return;
    
    bool delete_flag=false;
    
    // 如果强制删除或平面拟合失败，标记为删除
    if(ifdelete || node_input->plane_==NULL) 
        delete_flag=true;
    else
    {
        if(node_input->parent_!=NULL)
            // 检查与父节点间是否存在碰撞
            delete_flag=!world_->collisionFree(node_input,node_input->parent_);
        else
            // 如果是根节点，检查位置是否可通行
            delete_flag=!world_->isFree(node_input->position_);
    }
    
    // 如果节点无效，添加到无效节点列表
    if(delete_flag) invalid_nodes.push_back(node_input);
    
    // 递归检查所有子节点
    for(auto &node:node_input->children_) 
        addInvalidNodes(node,delete_flag,invalid_nodes);
}

/**
 * @brief 修剪树结构，删除无效节点
 * @description 删除树中无效的节点，并更新父子关系，保持树结构的有效性
 *              这在动态环境中非常重要，可以移除因环境变化而变得不可通行的节点
 */
// trimTree() 会删除树中无效的节点，并更新父子关系。
void PFRRTStar::trimTree()
{
    vector<Node*> invalid_nodes;
    
    // 从根节点开始查找所有无效节点
    addInvalidNodes(node_origin_,false,invalid_nodes);
    
    // 处理每个无效节点
    for(auto &node:invalid_nodes)
    {
        // 如果节点有父节点，从父节点的子节点列表中删除该节点
        if(node->parent_!=NULL) 
            deleteChildren(node->parent_,node);
        
        // 从树的节点列表中删除该节点
        for(vector<Node*>::iterator it=tree_.begin();it!=tree_.end();++it)
        {
            if(*it==node)
            {
                tree_.erase(it);
                break;
            }
        }
    }
    
    // 清理无效节点的内存
    clean_vector(invalid_nodes);
}

/**
 * @brief 继承树结构到新的根节点
 * @param new_root 新的根节点指针
 * @return 是否成功继承树结构
 * @description 尝试将新的根节点连接到现有树结构中，通过重新构造父子关系来继承之前的规划结果
 *              这样可以避免完全重新规划，提高效率
 */
// inheritTree() 会尝试将新的根节点插入到树中，并更新树的结构和成本。
bool PFRRTStar::inheritTree(Node* new_root)
{ 
    bool result=false;

    // 重新拟合所有节点的平面（考虑到网格地图可能更新）
    for(auto&node:tree_) fitPlane(node);
        
    // 修剪树结构，删除无效节点
    trimTree();

    float min_dis = INF;        // 最小距离初始化为无穷大
    Node* node_insert = NULL;   // 最佳插入节点
  
    // 在树中寻找距离新根节点最近且无碰撞的节点
    for(const auto&node:tree_) 
    { 
        float tmp_dis=EuclideanDistance(node,new_root);
        if(tmp_dis < min_dis && world_->collisionFree(node,new_root)) 
        {
            min_dis = tmp_dis;
            node_insert = node;
        }
    }

    // 如果找到合适的插入点，则重构树结构
    if(node_insert!=NULL)
    {
        result=true;
        Node* node=node_insert;
        vector<Node*> node_record;
        
        // 沿着父节点链向上记录路径
        while(node!=NULL)
        {
            node_record.push_back(node);
            node=node->parent_;
        }
        
        // 反转父子关系：将路径上的节点重新连接
        for(size_t i=node_record.size()-1;i>0;i--)
        {
            deleteChildren(node_record[i],node_record[i-1]);  // 删除原有父子关系
            node_record[i]->parent_=node_record[i-1];         // 设置新的父子关系
            node_record[i-1]->children_.push_back(node_record[i]);
        }
        
        // 连接新根节点到树
        new_root->children_.push_back(node_insert);
        node_insert->parent_=new_root;
        tree_.push_back(new_root);
        
        // 更新节点成本
        updateNode(new_root);
        node_origin_=new_root;

        // 寻找邻居节点并进行重连优化
        vector<pair<Node*,float>> neighbor_record;
        findNearNeighbors(node_origin_,neighbor_record);
        reWire(node_origin_,neighbor_record);
    }
    return result;
}

/**
 * @brief 继承现有路径到新的根节点
 * @param new_root 新的根节点指针
 * @param type 路径类型（Global全局路径或Sub子路径）
 * @return 是否成功继承路径
 * @description 尝试将新的根节点插入到现有路径中，通过重新构造路径来继承之前的规划结果
 *              如果成功插入，则返回true，否则返回false
 */
// inheritPath() 会尝试将新的根节点插入到路径中，并更新路径的结构和成本。
// 如果成功插入，则返回 true，否则返回 false。
bool PFRRTStar::inheritPath(Node* new_root,Path::Type type)
{
    bool result=false;
    
    // 检查当前路径类型是否与请求类型匹配
    if(path_.type_==type)
    {
        // 复制路径节点
        vector<Node*> tmp_nodes;
        for(size_t i = 0;i < path_.nodes_.size();i++)
        {
            Node* node_now=path_.nodes_[i];
            // 重新拟合平面并创建新节点
            tmp_nodes.push_back(fitPlane(node_now->plane_->init_coord));
            
            // 检查节点是否有效以及与前一个节点间是否无碰撞
            if(tmp_nodes[i]==NULL || (tmp_nodes.size()>1 && !world_->collisionFree(tmp_nodes[i],tmp_nodes[i-1])))
                return false;
            
            // 检查当前节点与新根节点的距离以及连通性
            if(EuclideanDistance(tmp_nodes[i],new_root) < inherit_threshold_ && 
               world_->collisionFree(tmp_nodes[i],new_root))
            {
                result=true;
                break;
            }
        }
        
        // 如果可以继承路径，重构路径结构
        if(result)
        {
            tmp_nodes.push_back(new_root);
            
            // 根据路径类型确定起始索引
            size_t start_index=(type==Path::Global?1:0);
            
            // 建立父子关系
            for(size_t i=start_index;i<tmp_nodes.size()-1;i++)
            {
                tmp_nodes[i]->parent_=tmp_nodes[i+1];
                tmp_nodes[i+1]->children_.push_back(tmp_nodes[i]);
            }
            
            // 清理并重建树和路径
            path_=Path();
            clean_vector(tree_);
            tree_.assign(tmp_nodes.begin()+start_index,tmp_nodes.end());          
            node_origin_=new_root;
            updateNode(node_origin_);
            generatePath();
        }
    }
    return result;
}

/**
 * @brief 生成随机数
 * @return 0到1之间的随机浮点数
 * @description 使用随机设备和梅森旋转算法生成高质量的随机数
 */
float PFRRTStar::getRandomNum()
{
    random_device rand_rd;                          // 随机设备
    mt19937 rand_gen(rand_rd());                   // 梅森旋转随机数生成器
    uniform_real_distribution<> rand_unif(0, 1.0); // 均匀分布[0,1]
    return rand_unif(rand_gen);
}

/**
 * @brief 在地图边界内随机生成2D点
 * @return 随机生成的2D点坐标
 * @description 在地图边界内随机生成一个二维点，作为RRT*算法的采样点
 */
// getRandom2DPoint() 会在地图的边界内随机生成一个二维点，作为 RRT* 的采样点。
Vector2d PFRRTStar::getRandom2DPoint()
{
    Vector3d lb = world_->getLowerBound();  // 获取地图下边界
    Vector3d ub = world_->getUpperBound();  // 获取地图上边界

    // 在X和Y方向上随机采样
    Vector2d rand_point=Vector2d( (ub(0)-lb(0))*getRandomNum()+lb(0),
                                  (ub(1)-lb(1))*getRandomNum()+lb(1));
    return rand_point;           
}

/**
 * @brief 在椭球区域内采样
 * @return 椭球区域内的随机3D点
 * @description 在起点和终点之间的椭球区域中生成采样点，这是一种启发式采样策略
 *              可以帮助RRT*更快地集中搜索可能包含最优路径的区域，提升规划效率
 */
// sampleInEllipsoid() 会在起点和终点之间的一个长椭球区域中生成一个三维点，帮助 RRT* 更快集中搜索目标路径区域，提升规划效率。
Vector3d PFRRTStar::sampleInEllipsoid()
{
    // 计算起点到终点的直线距离（最小可能路径长度）
    float cmin=EuclideanDistance(node_target_,node_origin_);
    
    // 计算从起点到终点的单位向量
    Vector3d a_1=(node_target_->position_-node_origin_->position_)/cmin;  
    RowVector3d id_t(1,0,0);  // 标准单位向量
    
    // 计算旋转矩阵的中间矩阵
    Matrix3d M=a_1*id_t;
    JacobiSVD<MatrixXd> SVD(M,ComputeFullU|ComputeFullV);  // SVD分解
    Matrix3d U=SVD.matrixU();   
    Matrix3d V=SVD.matrixV();
    
    // 构造旋转矩阵
    Matrix3d A=Matrix3d::Zero();
    A(0,0)=A(1,1)=1,A(2,2)=U.determinant()*V.determinant();
    Matrix3d C=U*A*V;

    // 当前最佳路径长度（椭球长半轴）
    float cbest=path_.dis_+1.0f;

    // 构造椭球的缩放矩阵
    Matrix3d L=Matrix3d::Zero();
    L(0,0)=cbest*0.5;  // 长半轴（沿起点-终点方向）
    L(1,1)=L(2,2)=sqrt(powf(cbest, 2)-powf(cmin,2))*0.5;  // 短半轴

    // 在单位球内随机采样（球坐标）
    float theta1=acos(2*getRandomNum()-1);  // 极角
    float theta2=2*PI*getRandomNum();       // 方位角
    float radius=powf(getRandomNum(),1.0/3); // 径向距离（保证均匀分布）

    Vector3d random_ball;
    random_ball << radius*sin(theta1)*cos(theta2),
                   radius*sin(theta1)*sin(theta2),
                   radius*cos(theta1);

    // 将单位球内的点变换到椭球内
    Vector3d random_ellipsoid=C*L*random_ball;

    // 椭球中心（起点和终点的中点）
    Vector3d center= (node_origin_->position_+node_target_->position_)*0.5;
    Vector3d point=random_ellipsoid+center;
    return point;
}

/**
 * @brief 在扇形区域内采样
 * @return 扇形区域内的随机2D点
 * @description 在Roll模式下，基于现有路径生成扇形采样区域
 *              这种采样策略可以帮助RRT*在滚动规划时更有效地搜索路径延续方向
 */
// sampleInSector() 会在路径的扇形区域内生成一个二维点，帮助 RRT* 在 Roll 状态下更快集中搜索目标路径区域，提升规划效率。
Vector2d PFRRTStar::sampleInSector()
{
    float sample_sector_lb_=2.0f;  // 扇形采样下界距离

    vector<float> theta_record;    // 记录路径方向角度

    Vector2d start_2D=project2plane(node_origin_->position_);
    
    // 计算路径中各点相对于起点的方向角
    for(size_t i=0;i<path_.nodes_.size()-1;i++)
    {
        Vector2d pt_2D=project2plane(path_.nodes_[i]->position_);
        Vector2d diff=pt_2D-start_2D;
        
        // 跳过太近的点
        if(diff.norm() < sample_sector_lb_) continue;
        
        float theta=atan2f(diff(1),diff(0));  // 计算方向角
        theta_record.push_back(theta);       
    }

    // 如果没有有效方向角，退回到随机采样
    if(theta_record.empty()) return getRandom2DPoint(); 

    default_random_engine engine;
    uniform_int_distribution<unsigned> rand_int(0,theta_record.size()-1);

    float theta_rate = getRandomNum();

    // 选择一个基准方向角并添加随机偏移（±20度内）
    float theta=theta_record[rand_int(engine)]+(theta_rate-0.5)*20.0*PI/180.0;

    Vector2d sub_goal_2D=project2plane(path_.nodes_.front()->position_);

    // 计算扇形采样上界距离
    float sample_sector_ub=(EuclideanDistance(start_2D,sub_goal_2D)+EuclideanDistance(start_2D,end_pos_2D_))*0.5+1.0f;

    float rand_num=getRandomNum();

    // 在扇形内随机采样距离
    float R=sqrt(rand_num)*(sample_sector_ub-sample_sector_lb_)+sample_sector_lb_;

    // 生成极坐标下的随机点
    Vector2d rand_point(R*cos(theta) , R*sin(theta));
    rand_point+=project2plane(node_origin_->position_);

    return rand_point;
}

/**
 * @brief 智能采样函数
 * @return 采样得到的2D点坐标
 * @description 根据当前规划状态选择不同的采样策略：
 *              - Global模式：椭球采样或目标偏向采样
 *              - Roll模式：扇形采样或随机采样  
 *              - WithoutGoal模式：纯随机采样
 *              这种自适应采样策略可以显著提高路径规划的效率
 */
// sample() 会根据当前的规划状态，生成一个二维采样点。
// 如果当前状态是 Global，则在起点和终点之间的椭球区域内采样；
// 如果是 Roll 状态，则在路径的扇形区域内采样；
// 如果是 WithoutGoal 状态，则随机生成一个二维点。
// 返回生成的二维采样点。
// 注意：此函数会根据规划状态的不同，调用不同的采样方法。
// 例如，在 Global 状态下调用 sampleInEllipsoid()，在 Roll 状态下调用 sampleInSector()。
// 如果规划状态无效，则返回一个空的二维点。
// 该函数的目的是为 RRT* 算法提供随机采样点，以便在规划过程中探索新的路径。
// 该函数的实现依赖于 project2plane() 函数将三维点投影到二维平面上。
// 该函数还会根据规划状态的不同，调整采样点的分布和范围，以提高路径规划的效率。
// 该函数的返回值是一个 Vector2d 类型的二维点，表示采样结果。
// 该函数的实现使用了随机数生成器和均匀分布，以确保采样点的随机性和均匀性。
Vector2d PFRRTStar::sample()
{
    Vector2d point_sample;
    switch(planning_state_)
    {
        case Global:  // 全局规划模式
        {
            if(!path_.nodes_.empty()) 
                // 如果已有路径，使用椭球采样（更智能的采样策略）
                point_sample=project2plane(sampleInEllipsoid());
            else
                // 如果没有路径，使用目标偏向采样或随机采样
                point_sample=(getRandomNum() < goal_biased_ )?project2plane(node_target_->position_):getRandom2DPoint();
        }
        break;
        case Roll:    // 滚动规划模式
            // 如果有路径使用扇形采样，否则使用随机采样
            point_sample=path_.nodes_.empty()?getRandom2DPoint():sampleInSector();
        break;
        case WithoutGoal:  // 无目标探索模式
            // 纯随机采样，用于自主探索
            point_sample=getRandom2DPoint();
        break;
        default:
        break;
    }
    return point_sample;
}

/**
 * @brief 寻找距离给定点最近的树节点
 * @param point 查询点的2D坐标
 * @return 树中距离查询点最近的节点指针
 * @description 在树中搜索距离给定2D点最近的节点，使用曼哈顿距离代替欧几里得距离以提高计算速度
 *              曼哈顿距离计算更快，在大多数情况下能提供足够好的近似结果
 */
// findNearest() 会在树中找到距离给定二维点最近的节点，返回该节点指针。
Node* PFRRTStar::findNearest(const Vector2d &point)
{
    float min_dis = INF;           // 最小距离初始化为无穷大
    Node* node_closest = NULL;     // 最近节点指针
  
    for(const auto&node:tree_) 
    { 
        // 使用曼哈顿距离代替欧几里得距离以提高计算速度
        float tmp_dis=fabs(point(0)-node->position_(0))+fabs(point(1)-node->position_(1));
        if(tmp_dis < min_dis) 
        {
            min_dis = tmp_dis;
            node_closest = node;
        }
    }
    return node_closest;
}

/**
 * @brief 控制扩展步长的导向函数
 * @param point_rand_projection 随机采样点的2D投影坐标
 * @param point_nearest_projection 最近节点的2D投影坐标
 * @return 经过步长限制后的新点2D坐标
 * @description 从最近点向随机点扩展，但限制扩展距离不超过设定的步长step_size_
 *              这样可以保证树的增长是渐进式的，避免跳跃式扩展导致的问题
 */
// steer() 会根据给定的随机点和最近点，计算出一个新的点，该点在两者之间，且距离最近点不超过 step_size_。
Vector2d PFRRTStar::steer(const Vector2d &point_rand_projection, const Vector2d &point_nearest_projection)
{
    Vector2d point_new_projection;
    Vector2d steer_p = point_rand_projection - point_nearest_projection;  // 扩展方向向量
    float steer_norm=steer_p.norm();  // 扩展距离
    
    // 检查扩展距离是否超过最大步长
    if (steer_norm > step_size_) 
        // 如果超过最大步长，则按比例缩放到最大步长
        point_new_projection = point_nearest_projection +  steer_p * step_size_/steer_norm;
    else 
        // 如果未超过最大步长，则直接使用随机点
        point_new_projection=point_rand_projection;
    return point_new_projection;
}

/**
 * @brief 在2D点上拟合平面并创建节点
 * @param p_original 原始2D坐标点
 * @return 拟合成功则返回新节点指针，失败返回NULL
 * @description 将2D点投影到地表面，然后在该位置拟合一个局部平面，创建包含平面信息的节点
 *              平面拟合是地形感知规划的关键步骤，用于分析地形的可通行性
 */
// fitPlane() 会将给定的二维点投影到表面上，并在该点上拟合一个平面，返回一个新的节点指针。
Node* PFRRTStar::fitPlane(const Vector2d &p_original)
{
    Node* node = NULL;
    Vector3d p_surface;
    
    // 确保2D点可以投影到地表面，否则返回空指针
    if(world_->project2surface(p_original(0),p_original(1),&p_surface))
    {
        node=new Node;
        // 在地表面点上拟合局部平面
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        // 机器人位置 = 地表面位置 + 离地高度 * 平面法向量
        node->position_ = p_surface + h_surf_ * node->plane_->normal_vector;
    }
    return node;
}

/**
 * @brief 重新拟合节点的平面并更新位置
 * @param node 需要重新拟合平面的节点指针
 * @description 删除节点原有的平面，基于初始坐标重新进行平面拟合并更新节点位置
 *              这在动态环境中很重要，当地形信息更新时需要重新计算节点的几何属性
 */
// fitPlane() 会将给定的节点的平面拟合到表面上，并更新节点的位置和成本。
void PFRRTStar::fitPlane(Node* node)
{
    Vector2d init_coord=node->plane_->init_coord;  // 保存初始坐标
    
    // 删除原有平面数据
    delete node->plane_;
    node->plane_=NULL;
    
    Vector3d p_surface;
    // 基于初始坐标重新投影到地表面
    if(world_->project2surface(init_coord(0),init_coord(1),&p_surface))
    {
        // 重新拟合平面
        node->plane_=new Plane(p_surface,world_,radius_fit_plane_,fit_plane_arg_);
        // 更新节点位置
        node->position_=p_surface + h_surf_ * node->plane_->normal_vector;
    }
}

/**
 * @brief 寻找新节点的近邻节点
 * @param node_new 新添加的节点指针
 * @param record 存储邻居节点和对应成本的记录（输出参数）
 * @description 在一定半径范围内找到所有与新节点距离较近且无碰撞的旧节点
 *              这些邻居节点将用于RRT*的路径优化（重新选择父节点和重连操作）
 */
// findNearNeighbors() 会在一定半径范围内找到所有靠近新节点的旧节点，供 RRT* 后续路径优化使用。
void PFRRTStar::findNearNeighbors(Node* node_new,vector<pair<Node*,float>> &record) 
{ 
    for (const auto&node:tree_) 
    {
        // 检查距离是否在邻居半径内，并且两节点间无碰撞
        if(EuclideanDistance(node_new,node) < neighbor_radius_ && world_->collisionFree(node_new,node) )
            // 记录邻居节点及其与新节点的连接成本
            record.push_back( pair<Node*,float>(node,calCostBetweenTwoNode(node_new,node)) );
    }
}

/**
 * @brief RRT*算法中的重新选择父节点操作  
 * @param node_new 新添加的节点
 * @param record 邻居节点及其连接成本的记录
 * @description 从邻居节点中选择能够提供最小总成本的节点作为新节点的父节点
 *              这是RRT*相比RRT的核心优化之一，能够动态选择最优父节点路径
 *              通过比较所有邻居节点的路径成本，选择最优的父节点连接
 */
// findParent() 会在记录的邻居节点中找到成本最小的父节点，并更新新节点的父节点和成本。
void PFRRTStar::findParent(Node* node_new,const vector<pair<Node*,float>> &record) 
{    
    Node* node_parent=NULL;  // 候选的最优父节点
    float min_cost = INF;    // 初始化最小成本为无穷大
    
    // 遍历所有邻居节点，寻找能提供最小成本路径的节点
    for(const auto&rec:record) 
    { 
        Node* node=rec.first;                    // 候选父节点
        float tmp_cost=node->cost_+rec.second;   // 通过该节点到达新节点的总成本
        
        // 如果找到更优路径，更新最小成本和对应的父节点
        if(tmp_cost < min_cost)
        {
            node_parent=node;
            min_cost=tmp_cost;
        }
    }
    
    // 设置新节点的父节点和成本
    node_new->parent_=node_parent;
    node_new->cost_=min_cost;
    // 将新节点添加到父节点的子节点列表中
    node_parent->children_.push_back(node_new);
}

/**
 * @brief RRT*算法中的重连操作
 * @param node_new 新添加的节点 
 * @param record 邻居节点及其连接成本的记录
 * @description 检查是否可以将新节点作为邻居节点的父节点来降低它们的路径成本
 *              这是RRT*的核心优化操作之一，通过重新连接现有节点来改进整体路径质量
 *              如果通过新节点的路径更短，就会重新连接并递归更新所有子节点的成本
 */
// reWire() 会遍历记录的邻居节点，并尝试将新节点作为它们的父节点，如果成本更低则更新。
// 该函数会删除原有的父子关系，并更新子节点的成本。
// 该函数的目的是优化路径，使得新节点能够更好地连接到已有的树结构中，从而提高路径规划的效率和质量。
void PFRRTStar::reWire(Node* node_new,const vector<pair<Node*,float>> &record) 
{ 
    for (const auto&rec:record) 
    { 
        Node* node=rec.first;                         // 候选重连的邻居节点
        float tmp_cost=node_new->cost_+rec.second;    // 如果新节点作为父节点的成本
        float costdifference=node->cost_-tmp_cost;    // 成本差异，正值表示新路径更优
        if(costdifference > 0)  // 如果通过新节点的路径成本更低
        {
            // 从原父节点的子节点列表中删除该节点
            deleteChildren(node->parent_,node);
            // 重新设置父子关系：将新节点设为该邻居节点的父节点
            node->parent_=node_new;
            node->cost_=tmp_cost;
            node_new->children_.push_back(node);
            // 递归更新该节点所有子节点的成本
            updateChildrenCost(node,costdifference);
        }
    }
}

/**
 * @brief 从父节点的子节点列表中删除指定子节点
 * @param parent 父节点指针
 * @param child 要删除的子节点指针
 * @description 维护树结构的完整性，当重连操作发生时需要正确更新父子关系
 *              使用迭代器安全地从vector中删除指定元素
 */
// deleteChildren() 会从父节点的子节点列表中删除指定的子节点，并更新树结构。
void PFRRTStar::deleteChildren(Node* node_parent,Node* node_children)
{
    // 遍历父节点的所有子节点，寻找要删除的节点
    for(vector<Node*>::iterator it=node_parent->children_.begin();it!=node_parent->children_.end();++it)
    {
        if(*it==node_children)  // 找到目标子节点
        {
            node_parent->children_.erase(it);  // 从子节点列表中删除
            break;  // 删除后退出循环
        }
    }
}

/**
 * @brief 递归更新子节点的成本
 * @param node_root 根节点（要更新的节点）
 * @param costdifference 成本差值，正值表示成本减少
 * @description 当节点的父节点变更导致路径成本改变时，需要递归更新该节点及其所有后代节点的成本
 *              这确保了整个子树的成本值都能正确反映新的路径成本
 *              使用深度优先遍历的方式递归处理所有子节点
 */
// updateChildrenCost() 会递归地更新子节点的成本，减去给定的成本差值。
// 该函数用于在路径优化过程中调整子节点的成本，使得新的父节点能够更好地连接到已有的树结构中。
// 通过更新子节点的成本，可以确保路径规划的效率和质量得到提升。
// 该函数的实现使用了递归算法，遍历子节点并更新其成本值。
// 该函数的参数包括父节点指针和成本差值，返回值为 void。
void PFRRTStar::updateChildrenCost(Node* &node_root, const float &costdifference) 
{
    // 遍历根节点的所有直接子节点
    for(auto&node:node_root->children_)
    {
        node->cost_ -= costdifference;              // 更新子节点成本（减去成本差值）
        updateChildrenCost(node,costdifference);    // 递归更新该子节点的所有后代
    }
}

/**
 * @brief 检查新节点是否接近目标，判断是否可以完成路径规划
 * @param node_new 新添加的节点
 * @param closerecord 接近目标的节点记录（输出参数）
 * @description 根据当前规划状态检查新节点是否足够接近目标以完成路径规划
 *              - Global状态：检查是否接近最终目标点
 *              - Roll状态：检查是否接近当前子目标点（投影到平面）
 *              如果距离满足阈值且路径无碰撞，则记录为可连接的目标节点
 */
// closeCheck() 会根据当前的规划状态，检查新节点是否接近目标节点，并记录在闭环检查记录中。
// 如果当前状态是 Global，则检查新节点到目标节点的距离是否小于目标阈值，并且两者之间没有障碍物；
// 如果是 Roll 状态，则检查新节点到投影到平面上的终点的距离是否小于子目标阈值。
// 如果满足条件，则将新节点和成本记录到闭环检查记录中。
// 该函数的目的是在路径规划过程中检查新节点是否接近目标节点，以便在满足条件时生成最终路径。
// 该函数的实现依赖于 EuclideanDistance() 函数计算两点之间的欧几里得距离，以及 world_->collisionFree() 函数检查两点之间是否有障碍物。
void PFRRTStar::closeCheck(Node* node)
{
    switch(planning_state_)  // 根据当前规划状态进行不同的检查
    {
        case Global:  // 全局路径规划状态
        {
            // 检查新节点是否足够接近最终目标且路径无碰撞
            if(EuclideanDistance(node,node_target_) < goal_threshold_ && world_->collisionFree(node,node_target_))
                // 记录可连接到目标的节点及其连接成本
                close_check_record_.push_back(pair<Node*,float>(node,calCostBetweenTwoNode(node,node_target_)));
        }
        break;
        case Roll:   // 滚动窗口规划状态
        {
            // 检查新节点投影到2D平面后是否接近当前子目标
            if(EuclideanDistance(project2plane(node->position_),end_pos_2D_) < sub_goal_threshold_)
                // 记录接近子目标的节点，成本使用距离的三次方（增加对距离的敏感度）
                close_check_record_.push_back(pair<Node*,float>(node,powf(EuclideanDistance(end_pos_2D_,project2plane(node->position_)),3)));
        }
        break;
        default:  // 其他状态不进行处理
        break;
    }
}

/**
 * @brief 计算路径的总长度
 * @param nodes 构成路径的节点序列
 * @return 路径总长度（所有相邻节点间距离之和）
 * @description 通过累加路径中所有相邻节点间的欧几里得距离来计算总路径长度
 *              用于评估和比较不同路径的质量，选择最优路径
 */
// calPathDis() 会计算给定节点列表的路径长度，即所有相邻节点之间的欧几里得距离之和。
// 该函数用于在路径规划过程中评估路径的长度，以便选择最优路径。
// 该函数的实现使用了 EuclideanDistance() 函数计算两点之间的距离，并将所有距离累加得到总路径长度。
// 该函数的参数是一个节点指针向量，返回值是一个浮点数，表示路径长度。
float PFRRTStar::calPathDis(const vector<Node*> &nodes)
{
    float dis=0.0f;  // 初始化路径总长度
    // 遍历路径中的相邻节点对，累加距离
    for(size_t i=0;i < nodes.size()-1; i++)
        dis+=EuclideanDistance(nodes[i],nodes[i+1]);
    return dis;  // 返回总路径长度
}

/**
 * @brief 生成最终路径
 * @description 根据当前规划状态从可连接到目标的节点中选择最优节点生成路径
 *              - Global状态：选择总成本最小的节点连接到最终目标
 *              - Roll状态：选择最优节点连接到当前子目标，并调整下次规划的参数
 *              通过回溯父节点关系构建完整路径，并计算路径统计信息
 */
// generatePath() 会根据当前的规划状态生成最终路径。
// 如果当前状态是 Global，则从闭环检查记录中选择成本最小的节点作为路径的起点，并将其添加到路径中；
// 如果是 Roll 状态，则从闭环检查记录中选择成本最小的节点作为子目标，并将其添加到路径中。
// 如果没有找到合适的节点，则根据当前信息调整子目标阈值，以便下次更容易找到解决方案。
// 最后，更新路径的成本和距离，并设置路径类型。
// 该函数的目的是在路径规划过程中生成最终路径，以便后续使用或可视化。
// 该函数的实现依赖于 close_check_record_ 成员变量存储的闭环检查记录，以及 calPathDis() 函数计算路径长度。
// 该函数的参数和返回值均为 void，表示不需要传入参数或返回值。
void PFRRTStar::generatePath()
{
    switch(planning_state_)  // 根据当前规划状态生成不同类型的路径
    {
        case Global:  // 全局路径规划状态
        {
            Node* node_choosed=NULL;   // 选定的最优节点
            float min_cost=path_.cost_; // 当前路径成本，用于比较
            
            // 从所有可连接到目标的节点中选择总成本最小的节点
            for (const auto&rec:close_check_record_) 
            {
                Node* node=rec.first;                    // 候选节点
                float tmp_cost=node->cost_+rec.second;   // 该节点到目标的总成本
                if(tmp_cost < min_cost)  // 如果找到更优路径
                {
                    min_cost=tmp_cost;
                    node_choosed=node;
                }
            }
            if(min_cost!=path_.cost_)  // 如果找到了更优的路径
            {
                // 清空当前路径，重新构建
                path_.nodes_.clear();
                path_.nodes_.push_back(node_target_);  // 路径起点是目标节点
                
                // 通过回溯父节点构建完整路径（从目标到起点）
                while(node_choosed!=NULL)
                {
                    path_.nodes_.push_back(node_choosed);
                    node_choosed=node_choosed->parent_;
                }
                
                // 更新路径统计信息
                path_.cost_=min_cost;                    // 更新路径成本
                path_.dis_=calPathDis(path_.nodes_);     // 计算路径长度
                path_.type_=Path::Global;                // 设置路径类型为全局路径
            }
        }
        break;
        case Roll:  // 滚动窗口路径规划状态  
        {
            path_=Path();          // 重置路径
            Node* sub_goal = NULL; // 选定的子目标节点
            float min_cost = INF;  // 最小成本初始化为无穷大
            
            // 从所有可连接的节点中选择成本最小的作为子目标
            for(const auto&rec:close_check_record_) 
            { 
                Node* node=rec.first;                    // 候选子目标节点
                float tmp_cost=node->cost_+rec.second;   // 到达该节点的总成本
                if (tmp_cost < min_cost) 
                {
                    min_cost = tmp_cost; 
                    sub_goal = node;
                }
            }
            
            if(sub_goal!=NULL)  // 如果找到了合适的子目标
            {
                // 通过回溯父节点构建到子目标的路径
                while(sub_goal!=NULL)
                {
                    path_.nodes_.push_back(sub_goal);
                    sub_goal=sub_goal->parent_;
                }
                
                // 设置路径统计信息
                path_.cost_=path_.nodes_.front()->cost_;  // 路径成本等于终点节点的成本
                path_.dis_=calPathDis(path_.nodes_);      // 计算路径长度
                path_.type_=Path::Sub;
            }
            else
            {
                //If close_check_record is empty,adjust the threshold according to the current information of distance
                //so that next time it will more likely to get a solution
                float min_dis=INF;
                for(const auto&node:tree_)
                {
                    float tmp_dis=EuclideanDistance(project2plane(node->position_),end_pos_2D_);
                    if(tmp_dis<min_dis) min_dis=tmp_dis;
                }
                sub_goal_threshold_=min_dis+1.0f;
            }
        }
        break;
        default:
        break;
    }
}

/**
 * @brief RRT*路径规划算法的主函数
 * @param max_iter 最大迭代次数
 * @param max_time 最大运行时间（毫秒）
 * @return 规划生成的路径
 * @description RRT*算法的核心实现，在指定的时间和迭代次数限制内寻找最优路径
 *              算法流程：随机采样 -> 寻找最近节点 -> 扩展新节点 -> 平面拟合 -> 
 *              邻居搜索 -> 选择父节点 -> 重连优化 -> 目标检查 -> 路径生成
 *              支持全局规划和滚动窗口规划两种模式
 */
// planner() 会执行 RRT* 算法的主循环，直到达到最大迭代次数或最大时间限制。
// 在每次迭代中，算法会采样一个随机点，找到距离该点最近的节点，并从该节点扩展到新的点。
// 如果新点有效且在地图范围内，则将其添加到树中，并尝试优化路径。
// 如果新点接近目标点，则生成最终路径。
// 最后，更新可视化信息并发布树的可遍历性信息。
// 该函数的参数包括最大迭代次数和最大时间限制，返回值为 Path 类型，表示生成的路径。
// 该函数的实现依赖于 sample()、findNearest()、steer()、fitPlane() 等辅助函数，以及 world_ 成员变量提供的地图信息。
// 该函数的目的是在给定的时间和迭代次数内找到一条从起点到终点的路径，并返回该路径供后续使用。
Path PFRRTStar::planner(const int &max_iter,const double &max_time)
{
    // 检查规划器状态是否有效
    if(planning_state_==Invalid)
    {
        ROS_ERROR("Illegal operation:the planner is at an invalid working state!!");
        return {};
    }
    
    double time_now=curr_time_;                    // 记录开始时间
    timeval start;gettimeofday(&start,NULL);      // 获取系统时间用于计时
    
    // RRT*主循环：在时间和迭代次数限制内持续规划
    while (curr_iter_ < max_iter && curr_time_ < max_time)
    {
        //Update current iteration
        curr_iter_++;  // 更新当前迭代次数

        //Update current time consuming
        // 计算当前消耗的时间
        timeval end;gettimeofday(&end,NULL);
        float ms=1000*(end.tv_sec-start.tv_sec)+0.001*(end.tv_usec-start.tv_usec);
        curr_time_=ms+time_now;

        //Sample to get a random 2D point
        // 步骤1：随机采样一个2D点
        Vector2d rand_point_2D=sample();

        //Find the nearest node to the random point
        // 步骤2：在现有树中找到离随机点最近的节点
        Node* nearest_node= findNearest(rand_point_2D);
        Vector2d nearest_point_2D = project2plane(nearest_node->position_);

        //Expand from the nearest point to the random point
        // 步骤3：从最近节点向随机点方向扩展（限制步长）
        Vector2d new_point_2D = steer(rand_point_2D,nearest_point_2D);

        //Based on the new 2D point,
        // 步骤4：基于2D新点进行三维平面拟合，获得3D节点
        Node* new_node = fitPlane(new_point_2D); 

        if( new_node!=NULL               // 1.平面拟合成功，返回有效节点
            &&world_->isInsideBorder(new_node->position_)  // 2.新节点位置在地图范围内
          ) 
        {
            //Get the set of the neighbors of the new node in the tree
            // 步骤5：寻找新节点的邻居节点（用于RRT*优化）
            vector<pair<Node*,float>> neighbor_record;
            findNearNeighbors(new_node,neighbor_record);

            //Select an appropriate parent node for the new node from the set.
            // 步骤6：从邻居节点中为新节点选择最优父节点
            if(!neighbor_record.empty()) findParent(new_node,neighbor_record);
            //Different from other RRT algorithm,it is posible that the new node is too far away from the whole tree.If
            //so,discard the new node.
            // 如果新节点太远离整个树（无邻居节点），则舍弃该节点
            else
            {
                delete new_node;
                continue;
            }

            //Add the new node to the tree
            // 步骤7：将新节点添加到搜索树中
            tree_.push_back(new_node);

            //Rewire the tree to optimize it
            // 步骤8：执行重连操作优化树结构
            reWire(new_node,neighbor_record);

            //Check if the new node is close enough to the goal
            // 步骤9：检查新节点是否足够接近目标
            closeCheck(new_node);

            // 如果是全局规划模式，立即尝试生成路径
            if(planning_state_==Global) generatePath();
        }
        else   // 如果新节点无效或超出边界
            delete new_node;  // 释放内存
    }

    // 如果是滚动窗口模式，在循环结束后生成路径
    if(planning_state_==Roll) generatePath();

    // 可视化搜索树和发布可遍历性信息
    visTree(tree_,tree_vis_pub_);          // 可视化生成的搜索树
    pubTraversabilityOfTree(tree_tra_pub_); // 发布树节点的可遍历性信息
    
    return path_;  // 返回规划得到的路径
}

/**
 * @brief 发布搜索树节点的可遍历性信息
 * @param tree_tra_pub 用于发布可遍历性信息的ROS发布器指针
 * @description 将搜索树中每个节点的三维坐标和对应的地形可遍历性值打包发布
 *              用于可视化和分析地形的可通过性，帮助理解路径规划的决策过程
 *              消息格式：[x1,y1,z1,traversability1, x2,y2,z2,traversability2, ...]
 */
// pubTraversabilityOfTree() 会将树中每个节点的位置信息和可遍历性信息发布到指定的发布器上。
// 该函数会遍历树中的所有节点，将每个节点的 x、y、z 坐标和对应的平面可遍历性值添加到消息中，并发布该消息。
// 如果发布器为空，则直接返回，不进行任何操作。
// 该函数的目的是将树的可遍历性信息可视化，以便后续分析或调试使用。
// 该函数的实现使用了 std_msgs::Float32MultiArray 消息类型，包含一个 data 数组，用于存储节点信息。
// 该函数的参数是一个 Publisher 指针，表示要发布的主题，返回值为 void，表示不需要返回任何值。
void PFRRTStar::pubTraversabilityOfTree(Publisher* tree_tra_pub)
{
    if(tree_tra_pub==NULL) return;  // 检查发布器是否有效
    
    Float32MultiArray msg;  // 创建浮点数组消息
    // 遍历搜索树中的所有节点
    for(const auto&node:tree_)
    {
        // 添加节点的三维坐标
        msg.data.push_back(node->position_(0));  // x坐标
        msg.data.push_back(node->position_(1));  // y坐标
        msg.data.push_back(node->position_(2));  // z坐标
        // 添加该位置的地形可遍历性值
        msg.data.push_back(node->plane_->traversability);
    }
    tree_tra_pub->publish(msg);  // 发布可遍历性信息消息
}
