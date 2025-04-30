#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <unsupported/Eigen/Splines>
#include <cmath>
#include <cstdlib>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

// 确保M_PI可用
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 轨迹处理相关头文件
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// 添加枚举类型定义不同的平滑方法
enum SmoothingMethod {
    NURBS,
    CUBIC_POLYNOMIAL
};

class JointTrajectoryNURBSNode : public rclcpp::Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    // 预定义的关节空间目标点
    std::vector<std::vector<double>> joint_targets_;

    // 记录轨迹执行数据
    struct TrajectoryData
    {
        std::vector<std::vector<double>> positions;     // 关节角度
        std::vector<std::vector<double>> velocities;    // 关节角速度
        std::vector<std::vector<double>> accelerations; // 关节角加速度
        std::vector<double> timestamps;                 // 时间戳
        std::vector<double> segment_durations;          // 每段轨迹执行时间
    };

    std::vector<TrajectoryData> execution_data_;

    // 在类中添加新的成员变量标识当前使用的方法
    SmoothingMethod current_method_;

    // 关节状态订阅
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub_;

    // 修改关节状态数据记录结构
    struct JointMonitorData
    {
        std::vector<double> timestamp;                                   // 时间戳
        std::map<std::string, std::vector<double>> actual_positions;     // 实际关节位置
        std::map<std::string, std::vector<double>> actual_velocities;    // 实际关节速度
        std::map<std::string, std::vector<double>> actual_accelerations; // 实际关节加速度
        std::map<std::string, std::vector<double>> desired_positions;    // 期望关节位置
        std::map<std::string, std::vector<double>> desired_velocities;   // 期望关节速度
    };

    JointMonitorData joint_monitor_data_;
    rclcpp::Time monitor_start_time_;
    bool is_monitoring_ = false;
    std::map<std::string, double> last_velocities_; // 用于计算加速度

public:
    JointTrajectoryNURBSNode() : Node("joint_trajectory_nurbs_node")
    {
        RCLCPP_INFO(this->get_logger(), "初始化节点...");

        // 等待必要的服务启动
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 初始化MoveGroupInterface - 这里是问题所在
        // 不要在构造函数中使用shared_from_this()
        // 改为创建一个单独的初始化方法
    }

    // 添加一个专门的初始化方法
    void initialize()
    {
        // 初始化MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "russ_group");

        // 初始化规划场景监视器
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(),
            "robot_description",
            this->get_name());

        if (!planning_scene_monitor_)
        {
            throw std::runtime_error("无法创建规划场景监视器");
        }

        // 启动监视器
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        // 等待规划场景更新
        RCLCPP_INFO(this->get_logger(), "等待规划场景更新...");
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 配置运动规划参数
        setPlannerConfig();

        // 生成示例关节目标点
        generateJointTargets();

        // 添加对控制器状态的订阅
        // 先检查topic是否存在
        RCLCPP_INFO(this->get_logger(), "尝试订阅控制器状态: /russ_group_controller/state");
        controller_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/russ_group_controller/state", 1, // 增加队列大小
            std::bind(&JointTrajectoryNURBSNode::controller_state_callback, this, std::placeholders::_1));

        // 初始化监控起始时间
        monitor_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "初始化完成，准备开始测试");
    }

    void setPlannerConfig()
    {
        // 设置规划器参数
        move_group_->setMaxVelocityScalingFactor(1.0);     // 速度缩放因子
        move_group_->setMaxAccelerationScalingFactor(1.0); // 加速度缩放因子

        // 打印当前设置（删除不存在的getter方法调用）
        RCLCPP_INFO(this->get_logger(), "已设置最大速度缩放因子: 1.0");
        RCLCPP_INFO(this->get_logger(), "已设置最大加速度缩放因子: 1.0");
    }

    void generateJointTargets()
    {
        // 角度/弧度制开关，true为角度制，false为弧度制
        bool use_degrees = true;
        
        // 获取当前关节角度作为起始点（仅用于确定关节数量）
        std::vector<double> current_joints;
        {
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                throw std::runtime_error("无法获取规划场景");
            }
            moveit::core::RobotState current_state = scene->getCurrentState();
            current_state.copyJointGroupPositions(move_group_->getName(), current_joints);
        }
        
        // 获取关节数量
        int joint_count = current_joints.size();
        RCLCPP_INFO(this->get_logger(), "机器人有 %d 个关节", joint_count);
        
        if (use_degrees) {
            // 角度制目标点（后续会转换为弧度）
            RCLCPP_INFO(this->get_logger(), "使用角度制目标点");
            
            // 目标点1：起始位置（角度）
            joint_targets_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            
            // 目标点2（角度）
            joint_targets_.push_back({-50.0, 40.0, 50.0, -60.0, 50.0, -30.0});
            
            // 目标点3（角度）
            joint_targets_.push_back({-30.0, 60.0, 20.0, -30.0, 70.0, -10.0});
            
            // 目标点4（角度）
            joint_targets_.push_back({-70.0, -30.0, -20.0, 0.0, 20.0, 20.0});
            
            // 目标点5（角度）
            joint_targets_.push_back({-20.0, 0.0, 90.0, 40.0, -30.0, 50.0});

            // 目标点6（角度）
            joint_targets_.push_back({110.0, -50.0, 130.0, 150.0, 110.0, 130.0});

            // 目标点7：回到起始位置（角度）
            joint_targets_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            
            // 将角度转换为弧度
            const double DEG_TO_RAD = M_PI / 180.0;
            for (auto& target : joint_targets_) {
                for (auto& value : target) {
                    value *= DEG_TO_RAD;
                }
            }
        } else {
            // 弧度制目标点（直接使用）
            RCLCPP_INFO(this->get_logger(), "使用弧度制目标点");
            
            // 目标点1：起始位置（弧度）
            joint_targets_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            
            // 目标点2（弧度）
            joint_targets_.push_back({0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
            
            // 目标点3（弧度）
            joint_targets_.push_back({0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
            
            // 目标点4（弧度）
            joint_targets_.push_back({0.3, 0.3, 0.3, 0.3, 0.3, 0.3});
            
            // 目标点5（弧度）
            joint_targets_.push_back({0.15, 0.15, 0.15, 0.15, 0.15, 0.15});
            
            // 目标点6：回到起始位置（弧度）
            joint_targets_.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        }
        
        // 确保所有目标点的关节数量一致
        for (auto& target : joint_targets_) {
            // 如果机器人关节数不是6，则调整目标点的尺寸
            if (target.size() != static_cast<size_t>(joint_count)) {
                RCLCPP_WARN(this->get_logger(), "目标点关节数 %ld 与机器人关节数 %d 不匹配，将自动调整", 
                           target.size(), joint_count);
                
                // 调整尺寸
                target.resize(joint_count, 0.0);
            }
        }

        // 打印所有目标点（以弧度显示）
        RCLCPP_INFO(this->get_logger(), "生成了 %ld 个关节空间目标点（弧度值）", joint_targets_.size());
        for (size_t i = 0; i < joint_targets_.size(); ++i) {
            std::stringstream ss;
            ss << "目标点 " << i << ": [";
            for (size_t j = 0; j < joint_targets_[i].size(); ++j) {
                ss << joint_targets_[i][j];
                if (j < joint_targets_[i].size() - 1)
                    ss << ", ";
            }
            ss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        }
        
        // 如果使用角度制，同时也打印角度值
        if (use_degrees) {
            RCLCPP_INFO(this->get_logger(), "目标点的角度值:");
            const double RAD_TO_DEG = 180.0 / M_PI;
            for (size_t i = 0; i < joint_targets_.size(); ++i) {
                std::stringstream ss;
                ss << "目标点 " << i << " (角度): [";
                for (size_t j = 0; j < joint_targets_[i].size(); ++j) {
                    ss << joint_targets_[i][j] * RAD_TO_DEG;
                    if (j < joint_targets_[i].size() - 1)
                        ss << ", ";
                }
                ss << "]";
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            }
        }
    }

    void runTests()
    {
        // 对每种方法执行测试
        for (int method = 0; method <= 1; ++method)
        {
            current_method_ = method == 0 ? NURBS : CUBIC_POLYNOMIAL;
            std::string method_name = method == 0 ? "NURBS" : "三次多项式";
            
            RCLCPP_INFO(this->get_logger(), "\n开始测试 %s 插值方法", method_name.c_str());
            
            // 只执行一次测试，不需要循环
            RCLCPP_INFO(this->get_logger(), "\n开始%s轨迹测试", method_name.c_str());

            // 清空当前测试的执行数据
            execution_data_.clear();
            
            // 开始监控关节状态
            startMonitoring();

            // 依次执行每对相邻目标点之间的轨迹
            for (size_t i = 0; i < joint_targets_.size() - 1; ++i)
            {
                RCLCPP_INFO(this->get_logger(), "执行从目标点 %ld 到目标点 %ld 的轨迹", i, i + 1);

                // 创建轨迹
                robot_trajectory::RobotTrajectory trajectory = 
                    current_method_ == NURBS ? 
                    createNURBSTrajectory(joint_targets_[i], joint_targets_[i + 1]) :
                    createCubicPolynomialTrajectory(joint_targets_[i], joint_targets_[i + 1]);

                // 执行轨迹并记录数据
                executeAndRecordTrajectory(trajectory, i);

                // 等待轨迹完成并稳定
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
            
            // 停止监控关节状态
            stopMonitoring();
            
            // 保存监控数据（使用0作为测试索引）
            saveMonitoringData(method_name, 0);

            // 保存本次测试的执行数据（使用0作为测试索引）
            saveExecutionData(0, method_name);
            
            // 创建可视化数据
            createVisualizationData(method_name);
            
            RCLCPP_INFO(this->get_logger(), "%s插值方法测试完成", method_name.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "所有测试完成");
    }

    robot_trajectory::RobotTrajectory createNURBSTrajectory(
        const std::vector<double> &start_joints,
        const std::vector<double> &end_joints)
    {
        // 创建包含起始点和目标点的控制点数组
        std::vector<std::vector<double>> control_points;

        // 添加起始点
        control_points.push_back(start_joints);

        // 添加中间控制点 (为了有足够的控制点使用5阶NURBS)
        // 生成四个中间控制点，通过线性插值并加入随机扰动
        for (int i = 1; i <= 4; ++i)
        {
            double t = static_cast<double>(i) / 5.0;
            std::vector<double> mid_point(start_joints.size());

            for (size_t j = 0; j < start_joints.size(); ++j)
            {
                // 线性插值
                mid_point[j] = start_joints[j] * (1.0 - t) + end_joints[j] * t;

                // 添加小的随机扰动 (±0.05 弧度) 以创建更自然的曲线
                double random_offset = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.1;
                mid_point[j] += random_offset;
            }

            control_points.push_back(mid_point);
        }

        // 添加目标点
        control_points.push_back(end_joints);

        // 现在有6个控制点 (起点 + 4个中间点 + 终点)，足够使用5阶NURBS

        // 获取机器人模型和组名称
        const moveit::core::RobotModelConstPtr &robot_model = move_group_->getRobotModel();
        const std::string &group_name = move_group_->getName();

        // 创建机器人轨迹对象
        robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);

        // 获取当前状态
        planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
        if (!scene)
        {
            throw std::runtime_error("无法获取规划场景");
        }
        moveit::core::RobotState current_state = scene->getCurrentState();

        // 为每个控制点创建机器人状态并添加到轨迹
        for (const auto &point : control_points)
        {
            moveit::core::RobotState state = current_state;
            state.setJointGroupPositions(group_name, point);
            trajectory.addSuffixWayPoint(state, 0.1); // 初始时间间隔设为0.1秒
        }

        // 应用NURBS平滑
        if (!smoothTrajectoryWithNURBS(trajectory))
        {
            RCLCPP_WARN(this->get_logger(), "NURBS平滑失败，使用原始轨迹");
        }

        return trajectory;
    }

    bool smoothTrajectoryWithNURBS(robot_trajectory::RobotTrajectory &trajectory)
    {
        try
        {
            // 获取轨迹点
            std::vector<std::vector<double>> joint_positions;
            std::vector<double> time_stamps;

            for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
            {
                const moveit::core::RobotState &waypoint = trajectory.getWayPoint(i);
                std::vector<double> joint_values;
                waypoint.copyJointGroupPositions(move_group_->getName(), joint_values);
                joint_positions.push_back(joint_values);
                time_stamps.push_back(trajectory.getWayPointDurationFromStart(i));
            }

            // 对每个关节应用非均匀有理B样条平滑
            size_t num_joints = joint_positions[0].size();
            size_t num_points = joint_positions.size();

            RCLCPP_INFO(this->get_logger(), "NURBS平滑: 轨迹点数量: %ld", num_points);

            // 5阶NURBS需要至少6个控制点
            size_t k = 6; // 5阶B样条(degree=5, order=6)

            // 如果点数不足，退化为3阶NURBS
            if (num_points < k)
            {
                k = 4; // 3阶B样条(degree=3, order=4)
                RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，5阶NURBS需要至少6个点，退化为3阶NURBS", num_points);

                // 检查是否满足3阶NURBS的最小点数要求
                if (num_points < k)
                {
                    RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，3阶NURBS需要至少%ld个点，使用原始轨迹", num_points, k);
                    return true;
                }
            }

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());

            // 保留原始轨迹的第一个点
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(0), 0.0);

            // 新轨迹的总点数
            size_t new_num_points = std::max(size_t(20), num_points * 3); // 更多的采样点

            // 为所有关节创建非均匀B样条节点向量
            Eigen::VectorXd knots(num_points + k);

            // 设置非均匀节点向量 - 基于弦长参数化
            std::vector<double> chord_lengths(num_points);
            chord_lengths[0] = 0.0;

            // 计算弦长
            for (size_t i = 1; i < num_points; ++i)
            {
                double dist = 0.0;
                for (size_t j = 0; j < num_joints; ++j)
                {
                    double diff = joint_positions[i][j] - joint_positions[i - 1][j];
                    dist += diff * diff;
                }
                chord_lengths[i] = chord_lengths[i - 1] + std::sqrt(dist);
            }

            // 归一化弦长
            double total_length = chord_lengths.back();
            if (total_length > 1e-8)
            {
                for (size_t i = 0; i < num_points; ++i)
                {
                    chord_lengths[i] /= total_length;
                }
            }

            // 设置第一个和最后一个节点的重复度（Clamped B-spline曲线）
            for (size_t i = 0; i < k; ++i)
            {
                knots(i) = 0.0;
                knots(num_points + k - 1 - i) = 1.0;
            }

            // 根据弦长设置中间节点
            for (size_t i = 1; i < num_points - 1; ++i)
            {
                knots(i + k - 1) = chord_lengths[i];
            }

            // 设置权重 - 可以针对不同控制点调整权重以影响曲线形状
            Eigen::VectorXd weights = Eigen::VectorXd::Ones(num_points);

            // 为关键点设置基于曲率的权重
            for (size_t i = 1; i < num_points - 1; ++i)
            {
                // 计算当前点的曲率 - 使用相邻三点来估计曲率
                std::vector<double> curvatures(num_joints, 0.0);
                double max_curvature = 0.0;

                for (size_t j = 0; j < num_joints; ++j)
                {
                    // 使用中心差分近似计算曲率
                    double prev = joint_positions[i - 1][j];
                    double curr = joint_positions[i][j];
                    double next = joint_positions[i + 1][j];

                    // 计算二阶差分作为曲率估计
                    double curvature = std::abs(next - 2.0 * curr + prev);
                    curvatures[j] = curvature;
                    max_curvature = std::max(max_curvature, curvature);
                }

                // 如果有显著曲率，增加权重
                if (max_curvature > 1e-4)
                {
                    // 曲率越大，权重越高，但设置一个合理上限
                    double curvature_weight = 1.0 + std::min(4.0, max_curvature * 10.0);
                    weights(i) = curvature_weight;
                }
            }

            // 为起点和终点设置高权重，确保曲线精确通过
            weights(0) = 5.0;              // 起点
            weights(num_points - 1) = 5.0; // 终点

            // 计算非均匀时间间隔的新轨迹点
            for (size_t i = 1; i < new_num_points - 1; ++i)
            {
                // 归一化参数
                double u = static_cast<double>(i) / (new_num_points - 1);

                // 创建新状态
                moveit::core::RobotState new_state(trajectory.getRobotModel());
                new_state = trajectory.getWayPoint(0); // 复制第一个点的状态结构

                // 计算新的关节值
                std::vector<double> new_joint_values(num_joints, 0.0);

                // 对每个关节应用NURBS插值
                for (size_t j = 0; j < num_joints; ++j)
                {
                    // 提取关节位置作为控制点
                    std::vector<double> control_points;
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        control_points.push_back(joint_positions[p][j]);
                    }

                    // 计算NURBS值（有理B样条）
                    double numerator = 0.0;
                    double denominator = 0.0;

                    // 只处理有效的控制点索引范围
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        double basis = nurbsBasisSafe(p, k - 1, u, knots);
                        numerator += basis * weights(p) * control_points[p];
                        denominator += basis * weights(p);
                    }

                    // 计算有理B样条值
                    if (denominator > 1e-8)
                    {
                        new_joint_values[j] = numerator / denominator;
                    }
                    else
                    {
                        new_joint_values[j] = control_points[0];
                    }
                }

                // 设置新状态的关节值
                new_state.setJointGroupPositions(move_group_->getName(), new_joint_values);

                // 计算时间，基于弦长参数化的非均匀时间间隔
                double t_param = evaluateNonUniformParameter(u, chord_lengths);
                double current_time = time_stamps.front() + t_param * (time_stamps.back() - time_stamps.front());

                // 计算时间增量并添加到平滑轨迹
                double prev_time = smooth_trajectory.getWayPointDurationFromStart(
                    smooth_trajectory.getWayPointCount() - 1);
                double dt = current_time - prev_time;

                // 添加到轨迹（确保时间增量为正）
                if (dt > 1e-6)
                {
                    smooth_trajectory.addSuffixWayPoint(new_state, dt);
                }
            }

            // 添加最后一个点
            double remaining_time = time_stamps.back() - smooth_trajectory.getWayPointDurationFromStart(
                                                             smooth_trajectory.getWayPointCount() - 1);
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(num_points - 1),
                                                std::max(0.001, remaining_time)); // 确保时间增量为正

            // 更新原轨迹
            trajectory = smooth_trajectory;

            // 使用MoveIt的时间参数化，确保动力学约束
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            if (!iptp.computeTimeStamps(trajectory, 1.0, 1.0))
            {
                RCLCPP_WARN(this->get_logger(), "时间参数化失败，使用简单时间分配");
            }

            RCLCPP_INFO(this->get_logger(), "NURBS平滑成功，生成了 %ld 个轨迹点",
                        trajectory.getWayPointCount());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "NURBS轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    // 安全版本的NURBS基函数计算
    double nurbsBasisSafe(int i, int p, double u, const Eigen::VectorXd &knots)
    {
        // 越界检查
        if (i < 0 || i + p + 1 >= knots.size())
        {
            return 0.0; // 越界返回0
        }

        // 0阶基函数
        if (p == 0)
        {
            // 确保索引在有效范围内
            if (i >= knots.size() - 1)
                return 0.0;

            return (u >= knots(i) && u < knots(i + 1)) ||
                           (u >= knots(i) && u <= knots(i + 1) && i == knots.size() - 2)
                       ? 1.0
                       : 0.0;
        }

        // 递归计算前检查索引边界
        if (i + p >= knots.size() || i + 1 >= knots.size() || i + p + 1 >= knots.size())
        {
            return 0.0; // 避免递归中的索引越界
        }

        // 递归计算
        double coef1 = 0.0, coef2 = 0.0;

        if (knots(i + p) - knots(i) > 1e-10) // 避免除以零
        {
            coef1 = (u - knots(i)) / (knots(i + p) - knots(i));
        }

        if (knots(i + p + 1) - knots(i + 1) > 1e-10) // 避免除以零
        {
            coef2 = (knots(i + p + 1) - u) / (knots(i + p + 1) - knots(i + 1));
        }

        // 递归调用
        return coef1 * nurbsBasisSafe(i, p - 1, u, knots) +
               coef2 * nurbsBasisSafe(i + 1, p - 1, u, knots);
    }

    // 计算非均匀参数
    double evaluateNonUniformParameter(double u, const std::vector<double> &chord_lengths)
    {
        if (chord_lengths.empty())
            return u;

        // 搜索对应区间
        size_t idx = 0;
        while (idx < chord_lengths.size() - 1 && u > chord_lengths[idx + 1])
        {
            idx++;
        }

        // 如果u超出范围，返回边界值
        if (idx >= chord_lengths.size() - 1)
        {
            return chord_lengths.back();
        }

        // 线性插值
        if (idx < chord_lengths.size() - 1)
        {
            double t0 = chord_lengths[idx];
            double t1 = chord_lengths[idx + 1];

            if (std::abs(t1 - t0) < 1e-8) // 避免除以零
            {
                return t0;
            }

            // 在区间内线性插值
            double alpha = (u - t0) / (t1 - t0);
            return t0 * (1.0 - alpha) + t1 * alpha;
        }

        return u; // 默认情况下返回原参数
    }

    void executeAndRecordTrajectory(const robot_trajectory::RobotTrajectory &trajectory, [[maybe_unused]] size_t segment_index)
    {
        // 将轨迹转换为MoveIt计划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        trajectory.getRobotTrajectoryMsg(plan.trajectory_);

        // 记录当前段的执行数据
        TrajectoryData segment_data;

        // 预先存储轨迹点信息
        const auto &trajectory_pts = plan.trajectory_.joint_trajectory.points;
        for (const auto &pt : trajectory_pts)
        {
            segment_data.positions.push_back(pt.positions);
            segment_data.velocities.push_back(pt.velocities);
            segment_data.accelerations.push_back(pt.accelerations);
            segment_data.timestamps.push_back(pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9);
        }

        // 记录执行时间
        auto start_time = std::chrono::high_resolution_clock::now();

        // 执行轨迹
        RCLCPP_INFO(this->get_logger(), "开始执行轨迹，共 %ld 个点", trajectory_pts.size());
        auto exec_result = move_group_->execute(plan);

        // 计算执行时间
        auto end_time = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double>(end_time - start_time).count();

        // 记录段执行总时间
        segment_data.segment_durations.push_back(duration);

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "轨迹执行成功，耗时: %.3f 秒", duration);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "轨迹执行失败，错误码: %i",
                         static_cast<int>(exec_result.val));
        }

        // 添加到执行数据集合
        execution_data_.push_back(segment_data);
    }

    void saveExecutionData(int test_index, const std::string& method_name)
    {
        // 创建保存目录
        std::string dir_name = std::string("src/planning_node/test_results/") +
                               (current_method_ == NURBS ? "nurbs_joint_data" : "cubic_polynomial_joint_data");
        std::filesystem::path dir_path(dir_name);
        if (!std::filesystem::exists(dir_path))
        {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "创建目录: %s",
                       std::filesystem::absolute(dir_path).string().c_str());
        }

        // 构造文件名
        std::stringstream ss;
        ss << dir_name << "/joint_trajectory_test_" << test_index + 1 << ".csv";
        std::string filename = ss.str();

        // 创建CSV文件
        std::ofstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", filename.c_str());
            return;
        }

        // 获取关节名称
        std::vector<std::string> joint_names = move_group_->getJointNames();
        int joint_count = joint_names.size();

        // 写入CSV文件头
        file << "Segment,Point,Time(s),";
        for (int i = 0; i < joint_count; ++i)
        {
            file << joint_names[i] << "_Position(rad),";
            file << joint_names[i] << "_Velocity(rad/s),";
            file << joint_names[i] << "_Acceleration(rad/s^2),";
        }
        file << "Duration(s)\n";

        // 写入数据
        for (size_t segment = 0; segment < execution_data_.size(); ++segment)
        {
            const auto& segment_data = execution_data_[segment];
            
            // 写入每个轨迹点的数据
            for (size_t point = 0; point < segment_data.timestamps.size(); ++point)
            {
                file << segment + 1 << "," << point + 1 << "," << std::fixed << std::setprecision(6) << segment_data.timestamps[point] << ",";
                
                // 写入关节位置、速度和加速度
                for (int joint = 0; joint < joint_count; ++joint)
                {
                    // 防止索引越界
                    if (joint < static_cast<int>(segment_data.positions[point].size()))
                        file << segment_data.positions[point][joint] << ",";
                    else
                        file << "0.0,";
                    
                    if (!segment_data.velocities.empty() && joint < static_cast<int>(segment_data.velocities[point].size()))
                        file << segment_data.velocities[point][joint] << ",";
                    else
                        file << "0.0,";
                    
                    if (!segment_data.accelerations.empty() && joint < static_cast<int>(segment_data.accelerations[point].size()))
                        file << segment_data.accelerations[point][joint] << ",";
                    else
                        file << "0.0,";
                }
                
                // 写入段执行时间 (只在每段的最后一个点写入)
                if (point == segment_data.timestamps.size() - 1 && !segment_data.segment_durations.empty())
                    file << segment_data.segment_durations[0] << "\n";
                else
                    file << "0.0\n";
            }
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "%s轨迹数据已保存到: %s", method_name.c_str(), std::filesystem::absolute(filename).string().c_str());

        // 构造汇总文件名
        std::stringstream summary_ss;
        summary_ss << dir_name << "/summary_test_" << test_index + 1 << ".csv";
        std::string summary_filename = summary_ss.str();
        std::ofstream summary_file(summary_filename);

        if (summary_file.is_open())
        {
            summary_file << "Segment,Start Point,End Point,Duration(s)\n";
            
            for (size_t segment = 0; segment < execution_data_.size(); ++segment)
            {
                const auto& segment_data = execution_data_[segment];
                
                // 写入段摘要信息
                summary_file << segment + 1 << ",";
                
                // 写入起始点信息
                if (!segment_data.positions.empty() && !segment_data.positions[0].empty())
                {
                    summary_file << "[";
                    for (size_t j = 0; j < segment_data.positions[0].size(); ++j)
                    {
                        summary_file << std::fixed << std::setprecision(4) << segment_data.positions[0][j];
                        if (j < segment_data.positions[0].size() - 1)
                            summary_file << ",";
                    }
                    summary_file << "],";
                }
                else
                {
                    summary_file << "[],";
                }
                
                // 写入终点信息
                if (!segment_data.positions.empty() && !segment_data.positions.back().empty())
                {
                    summary_file << "[";
                    for (size_t j = 0; j < segment_data.positions.back().size(); ++j)
                    {
                        summary_file << std::fixed << std::setprecision(4) << segment_data.positions.back()[j];
                        if (j < segment_data.positions.back().size() - 1)
                            summary_file << ",";
                    }
                    summary_file << "],";
                }
                else
                {
                    summary_file << "[],";
                }
                
                // 写入执行时间
                if (!segment_data.segment_durations.empty())
                    summary_file << segment_data.segment_durations[0] << "\n";
                else
                    summary_file << "0.0\n";
            }
            
            summary_file.close();
            RCLCPP_INFO(this->get_logger(), "%s汇总数据已保存到: %s", method_name.c_str(), std::filesystem::absolute(summary_filename).string().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建汇总文件: %s", summary_filename.c_str());
        }
    }

    // 创建可视化数据文件，便于后续绘图
    void createVisualizationData(const std::string& method_name)
    {
        // 如果没有测试数据，则返回
        if (execution_data_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "没有可视化数据可供创建");
            return;
        }

        // 构造文件名
        std::string dir_name = std::string("src/planning_node/test_results/") +
                               (current_method_ == NURBS ? "nurbs_joint_data" : "cubic_polynomial_joint_data");
        std::string filename = dir_name + "/visualization_data.csv";

        // 创建CSV文件
        std::ofstream file(filename);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建可视化数据文件: %s", filename.c_str());
            return;
        }

        // 获取关节名称
        std::vector<std::string> joint_names = move_group_->getJointNames();
        
        // 写入CSV文件头
        file << "Time(s),";
        for (const auto& name : joint_names)
        {
            file << name << "_Position(rad),";
            file << name << "_Velocity(rad/s),";
            file << name << "_Acceleration(rad/s^2),";
        }
        file << "Segment\n";

        // 记录累计时间
        double cumulative_time = 0.0;
        
        // 写入所有段的数据
        for (size_t segment = 0; segment < execution_data_.size(); ++segment)
        {
            const auto& segment_data = execution_data_[segment];
            
            for (size_t point = 0; point < segment_data.timestamps.size(); ++point)
            {
                // 计算绝对时间
                double absolute_time = cumulative_time + segment_data.timestamps[point];
                file << std::fixed << std::setprecision(6) << absolute_time << ",";
                
                // 写入所有关节的数据
                for (size_t joint = 0; joint < joint_names.size(); ++joint)
                {
                    // 防止索引越界
                    if (joint < segment_data.positions[point].size())
                        file << segment_data.positions[point][joint] << ",";
                    else
                        file << "0.0,";
                    
                    if (!segment_data.velocities.empty() && joint < segment_data.velocities[point].size())
                        file << segment_data.velocities[point][joint] << ",";
                    else
                        file << "0.0,";
                    
                    if (!segment_data.accelerations.empty() && joint < segment_data.accelerations[point].size())
                        file << segment_data.accelerations[point][joint] << ",";
                    else
                        file << "0.0,";
                }
                
                file << segment + 1 << "\n";
            }
            
            // 更新累计时间
            if (!segment_data.segment_durations.empty())
                cumulative_time += segment_data.segment_durations[0];
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "%s可视化数据已保存到: %s", method_name.c_str(), std::filesystem::absolute(filename).string().c_str());
    }

    // 添加新的三次多项式轨迹生成方法
    robot_trajectory::RobotTrajectory createCubicPolynomialTrajectory(
        const std::vector<double> &start_joints,
        const std::vector<double> &end_joints)
    {
        // 创建包含起始点和目标点的控制点数组
        std::vector<std::vector<double>> control_points;

        // 添加起始点
        control_points.push_back(start_joints);

        // 添加中间控制点，三次多项式至少需要4个点
        // 生成两个中间控制点，通过线性插值
        for (int i = 1; i <= 2; ++i)
        {
            double t = static_cast<double>(i) / 3.0;
            std::vector<double> mid_point(start_joints.size());

            for (size_t j = 0; j < start_joints.size(); ++j)
            {
                // 线性插值
                mid_point[j] = start_joints[j] * (1.0 - t) + end_joints[j] * t;
            }

            control_points.push_back(mid_point);
        }

        // 添加目标点
        control_points.push_back(end_joints);

        // 获取机器人模型和组名称
        const moveit::core::RobotModelConstPtr &robot_model = move_group_->getRobotModel();
        const std::string &group_name = move_group_->getName();

        // 创建机器人轨迹对象
        robot_trajectory::RobotTrajectory trajectory(robot_model, group_name);

        // 获取当前状态
        planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
        if (!scene)
        {
            throw std::runtime_error("无法获取规划场景");
        }
        moveit::core::RobotState current_state = scene->getCurrentState();

        // 为每个控制点创建机器人状态并添加到轨迹
        for (const auto &point : control_points)
        {
            moveit::core::RobotState state = current_state;
            state.setJointGroupPositions(group_name, point);
            trajectory.addSuffixWayPoint(state, 0.1); // 初始时间间隔设为0.1秒
        }

        // 应用三次多项式平滑
        if (!smoothTrajectoryWithCubicPolynomial(trajectory))
        {
            RCLCPP_WARN(this->get_logger(), "三次多项式平滑失败，使用原始轨迹");
        }

        return trajectory;
    }

    // 添加三次多项式平滑方法
    bool smoothTrajectoryWithCubicPolynomial(robot_trajectory::RobotTrajectory &trajectory)
    {
        try
        {
            // 获取轨迹点
            std::vector<std::vector<double>> joint_positions;
            std::vector<double> time_stamps;

            for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
            {
                const moveit::core::RobotState &waypoint = trajectory.getWayPoint(i);
                std::vector<double> joint_values;
                waypoint.copyJointGroupPositions(move_group_->getName(), joint_values);
                joint_positions.push_back(joint_values);
                time_stamps.push_back(trajectory.getWayPointDurationFromStart(i));
            }

            // 对每个关节应用三次多项式平滑
            size_t num_joints = joint_positions[0].size();
            size_t num_points = joint_positions.size();

            RCLCPP_INFO(this->get_logger(), "三次多项式平滑: 轨迹点数量: %ld", num_points);

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());

            // 保留原始轨迹的第一个点
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(0), 0.0);

            // 新轨迹的总点数
            size_t new_num_points = std::max(size_t(20), num_points * 3); // 更多的采样点

            // 对每个关节计算三次多项式系数
            for (size_t i = 0; i < num_points - 1; ++i)
            {
                double segment_duration = time_stamps[i + 1] - time_stamps[i];
                if (segment_duration <= 0)
                    continue;
                
                for (size_t j = 1; j <= (new_num_points / (num_points - 1)); ++j)
                {
                    // 归一化参数 t 在 [0, 1] 区间内
                    double t = static_cast<double>(j) / (new_num_points / (num_points - 1));
                    double t_squared = t * t;
                    double t_cubed = t_squared * t;
                    
                    // 三次多项式系数，使用 Hermite 形式: p(t) = (2t³-3t²+1)p₀ + (t³-2t²+t)v₀ + (-2t³+3t²)p₁ + (t³-t²)v₁
                    double h00 = 2.0 * t_cubed - 3.0 * t_squared + 1.0;  // p₀系数
                    double h10 = t_cubed - 2.0 * t_squared + t;          // v₀系数
                    double h01 = -2.0 * t_cubed + 3.0 * t_squared;       // p₁系数
                    double h11 = t_cubed - t_squared;                    // v₁系数
                    
                    // 创建新状态
                    moveit::core::RobotState new_state(trajectory.getRobotModel());
                    new_state = trajectory.getWayPoint(0); // 复制第一个点的状态结构
                    
                    // 计算新的关节值
                    std::vector<double> new_joint_values(num_joints, 0.0);
                    
                    for (size_t k = 0; k < num_joints; ++k)
                    {
                        // 当前段的起点和终点
                        double p0 = joint_positions[i][k];
                        double p1 = joint_positions[i + 1][k];
                        
                        // 速度估计（可以使用简单的有限差分）
                        double v0 = 0.0, v1 = 0.0;
                        if (i > 0)
                            v0 = (p0 - joint_positions[i - 1][k]) / segment_duration;
                        if (i < num_points - 2)
                            v1 = (joint_positions[i + 2][k] - p1) / segment_duration;
                        
                        // 计算插值点
                        new_joint_values[k] = h00 * p0 + h10 * v0 * segment_duration + h01 * p1 + h11 * v1 * segment_duration;
                    }
                    
                    // 设置新状态的关节值
                    new_state.setJointGroupPositions(move_group_->getName(), new_joint_values);
                    
                    // 计算时间
                    double current_time = time_stamps[i] + t * segment_duration;
                    
                    // 计算时间增量并添加到平滑轨迹
                    double prev_time = smooth_trajectory.getWayPointDurationFromStart(
                        smooth_trajectory.getWayPointCount() - 1);
                    double dt = current_time - prev_time;
                    
                    // 添加到轨迹（确保时间增量为正）
                    if (dt > 1e-6)
                    {
                        smooth_trajectory.addSuffixWayPoint(new_state, dt);
                    }
                }
            }

            // 添加最后一个点
            double remaining_time = time_stamps.back() - smooth_trajectory.getWayPointDurationFromStart(
                                                              smooth_trajectory.getWayPointCount() - 1);
            if (remaining_time > 1e-6)
            {
                smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(num_points - 1), remaining_time);
            }

            // 更新原轨迹
            trajectory = smooth_trajectory;

            // 使用MoveIt的时间参数化，确保动力学约束
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            if (!iptp.computeTimeStamps(trajectory, 1.0, 1.0))
            {
                RCLCPP_WARN(this->get_logger(), "时间参数化失败，使用简单时间分配");
            }

            RCLCPP_INFO(this->get_logger(), "三次多项式平滑成功，生成了 %ld 个轨迹点",
                        trajectory.getWayPointCount());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "三次多项式轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    // 添加关节状态回调函数
    void controller_state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        // 添加调试信息
        static int message_count = 0;
        if (message_count % 100 == 0) { // 每100条消息打印一次，避免日志过多
            RCLCPP_INFO(this->get_logger(), "收到控制器状态消息 #%d，关节数：%ld", 
                       message_count, msg->joint_names.size());
        }
        message_count++;
        
        if (!is_monitoring_)
            return;

        // 计算相对于监控开始的时间
        double time_from_start = (this->now() - monitor_start_time_).seconds();
        joint_monitor_data_.timestamp.push_back(time_from_start);

        // 记录关节位置、速度和加速度（实际值和期望值）
        for (size_t i = 0; i < msg->joint_names.size(); ++i)
        {
            const std::string &joint_name = msg->joint_names[i];

            // 记录实际位置
            if (i < msg->actual.positions.size())
            {
                joint_monitor_data_.actual_positions[joint_name].push_back(msg->actual.positions[i]);
            }

            // 记录实际速度
            if (i < msg->actual.velocities.size())
            {
                joint_monitor_data_.actual_velocities[joint_name].push_back(msg->actual.velocities[i]);

                // 计算加速度（速度差分）
                if (joint_monitor_data_.timestamp.size() > 1 &&
                    last_velocities_.find(joint_name) != last_velocities_.end())
                {
                    double dt = joint_monitor_data_.timestamp.back() -
                                joint_monitor_data_.timestamp[joint_monitor_data_.timestamp.size() - 2];
                    if (dt > 0)
                    {
                        double acc = (msg->actual.velocities[i] - last_velocities_[joint_name]) / dt;
                        joint_monitor_data_.actual_accelerations[joint_name].push_back(acc);
                    }
                    else
                    {
                        joint_monitor_data_.actual_accelerations[joint_name].push_back(0.0);
                    }
                }
                else
                {
                    joint_monitor_data_.actual_accelerations[joint_name].push_back(0.0);
                }

                // 更新上一次速度
                last_velocities_[joint_name] = msg->actual.velocities[i];
            }

            // 记录期望位置
            if (i < msg->desired.positions.size())
            {
                joint_monitor_data_.desired_positions[joint_name].push_back(msg->desired.positions[i]);
            }

            // 记录期望速度
            if (i < msg->desired.velocities.size())
            {
                joint_monitor_data_.desired_velocities[joint_name].push_back(msg->desired.velocities[i]);
            }
        }
    }

    // 添加开始和停止监控的方法
    void startMonitoring()
    {
        // 清空监控数据
        joint_monitor_data_.timestamp.clear();
        joint_monitor_data_.actual_positions.clear();
        joint_monitor_data_.actual_velocities.clear();
        joint_monitor_data_.actual_accelerations.clear();
        joint_monitor_data_.desired_positions.clear();
        joint_monitor_data_.desired_velocities.clear();
        last_velocities_.clear();

        // 重置监控开始时间
        monitor_start_time_ = this->now();
        is_monitoring_ = true;
        RCLCPP_INFO(this->get_logger(), "开始监控控制器状态");
    }

    void stopMonitoring()
    {
        is_monitoring_ = false;
        RCLCPP_INFO(this->get_logger(), "停止监控关节状态");
    }

    // 添加保存监控数据的方法
    void saveMonitoringData(const std::string& method_name, int test_index)
    {
        if (joint_monitor_data_.timestamp.empty()) {
            RCLCPP_WARN(this->get_logger(), "没有监控数据可供保存");
            return;
        }
        
        std::string dir_name = std::string("src/planning_node/test_results/") +
                             (current_method_ == NURBS ? "nurbs_joint_data" : "cubic_polynomial_joint_data");
        std::filesystem::path dir_path(dir_name);
        if (!std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "创建目录: %s", std::filesystem::absolute(dir_path).string().c_str());
        }
        
        // 构造文件名
        std::string filename = dir_name + "/controller_state_test_" + std::to_string(test_index + 1) + ".csv";
        
        // 创建CSV文件
        std::ofstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", filename.c_str());
            return;
        }
        
        // 获取所有关节名称
        std::set<std::string> joint_names;
        for (const auto& pair : joint_monitor_data_.actual_positions) {
            joint_names.insert(pair.first);
        }
        
        // 写入CSV文件头
        file << "Time(s),";
        for (const auto& name : joint_names) {
            file << name << "_ActualPosition(rad),";
            file << name << "_ActualVelocity(rad/s),";
            file << name << "_ActualAcceleration(rad/s^2),";
            file << name << "_DesiredPosition(rad),";
            file << name << "_DesiredVelocity(rad/s),";
        }
        file << "\n";
        
        // 写入数据
        for (size_t i = 0; i < joint_monitor_data_.timestamp.size(); ++i) {
            file << std::fixed << std::setprecision(6) << joint_monitor_data_.timestamp[i] << ",";
            
            for (const auto& name : joint_names) {
                // 写入实际位置
                if (i < joint_monitor_data_.actual_positions[name].size()) {
                    file << joint_monitor_data_.actual_positions[name][i] << ",";
                } else {
                    file << "0.0,";
                }
                
                // 写入实际速度
                if (i < joint_monitor_data_.actual_velocities[name].size()) {
                    file << joint_monitor_data_.actual_velocities[name][i] << ",";
                } else {
                    file << "0.0,";
                }
                
                // 写入实际加速度
                if (i < joint_monitor_data_.actual_accelerations[name].size()) {
                    file << joint_monitor_data_.actual_accelerations[name][i] << ",";
                } else {
                    file << "0.0,";
                }
                
                // 写入期望位置
                if (i < joint_monitor_data_.desired_positions[name].size()) {
                    file << joint_monitor_data_.desired_positions[name][i] << ",";
                } else {
                    file << "0.0,";
                }
                
                // 写入期望速度
                if (i < joint_monitor_data_.desired_velocities[name].size()) {
                    file << joint_monitor_data_.desired_velocities[name][i] << ",";
                } else {
                    file << "0.0,";
                }
            }
            file << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "%s控制器状态数据已保存到: %s", 
                   method_name.c_str(), std::filesystem::absolute(filename).string().c_str());
    }
};

int main(int argc, char** argv)
{
    // 初始化ROS节点
    rclcpp::init(argc, argv);
    
    // 设置随机数种子，使得随机生成的轨迹具有一定重复性
    srand(static_cast<unsigned int>(time(nullptr)));
    
    try
    {
        // 创建节点实例
        auto node = std::make_shared<JointTrajectoryNURBSNode>();
        
        // 先初始化节点
        node->initialize();
        
        // 运行测试
        node->runTests();
        
        RCLCPP_INFO(node->get_logger(), "所有测试已完成，数据已保存");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("joint_trajectory_nurbs_node"), "发生错误: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    // 关闭ROS
    rclcpp::shutdown();
    return 0;
}