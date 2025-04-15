#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <filesystem>

// 轨迹处理相关头文件
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <unsupported/Eigen/Splines>

// 添加 planning_scene_monitor 头文件
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

/*
执行文件总耗时约25～30分钟
*/

using moveit::core::MoveItErrorCode;

// 平滑方法枚举
enum SmoothingMethod
{
    NONE,             // 无平滑
    MOVEIT_TOTG,      // MoveIt时间最优轨迹生成
    CUBIC_POLYNOMIAL, // 三次多项式
    B_SPLINE_3,       // 3阶B样条曲线
    B_SPLINE_5,       // 5阶B样条曲线
    NURBS             // NURBS曲线
};

// 添加 PathQualityMetrics 结构体定义
struct PathQualityMetrics {
    double path_length;      // 路径长度
    double joint_movement;   // 关节运动量
    double smoothness;       // 平滑度
    int waypoint_count;      // 路径点数量
    double min_clearance;    // 最小避障距离
    double execution_time;   // 执行时间
};

class TrajectorySmoothingTest : public rclcpp::Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<geometry_msgs::msg::Pose> test_poses_;

    // 不同平滑方法的性能指标
    struct SmoothingResults
    {
        std::vector<double> planning_times;
        std::vector<double> smoothing_times;
        std::vector<PathQualityMetrics> path_metrics;
    };

    std::map<SmoothingMethod, SmoothingResults> results_;

    // 添加退化计数器
    int bspline5_degraded_count_ = 0;
    int nurbs_degraded_count_ = 0;

    // 添加规划场景监视器
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:
    TrajectorySmoothingTest(const std::string &node_name)
        : Node(node_name)
    {
    }

    void initialize()
    {
        // 等待必要的服务启动
        rclcpp::sleep_for(std::chrono::seconds(2));

        // 初始化 MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "russ_group");

        // 初始化规划场景监视器
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(),         // ROS 节点
            "robot_description",        // 机器人描述参数
            this->get_name()           // 节点名称
        );

        if (!planning_scene_monitor_) {
            throw std::runtime_error("无法创建规划场景监视器");
        }

        // 启动监视器
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        // 等待规划场景更新
        RCLCPP_INFO(this->get_logger(), "等待规划场景更新...");
        rclcpp::sleep_for(std::chrono::seconds(2));

        setPlannerConfig();

        // 打印规划器信息
        printPlannerInfo();

        generatePoses();

        // 初始化结果容器
        results_[NONE] = SmoothingResults();
        results_[MOVEIT_TOTG] = SmoothingResults();
        results_[CUBIC_POLYNOMIAL] = SmoothingResults();
        results_[B_SPLINE_3] = SmoothingResults();
        results_[B_SPLINE_5] = SmoothingResults();
        results_[NURBS] = SmoothingResults();

        RCLCPP_INFO(this->get_logger(), "初始化完成，准备开始测试");
    }

    void generatePoses()
    {
        for (int i = 0; i < 50; i++)
        {
            geometry_msgs::msg::Pose pose;
            // 在工作空间内生成圆形轨迹上的位姿
            pose.position.x = 0. + 0.13 * cos(i * 2 * M_PI / 50);
            pose.position.y = 0.9 + 0.13 * sin(i * 2 * M_PI / 50);
            pose.position.z = 1.4;
            // 设置固定方向
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;

            test_poses_.push_back(pose);
        }
    }

    void runTest()
    {
        // 对每种平滑方法进行测试
        for (const auto& method_pair : {
            std::make_pair(NONE, "线性插值"),
            std::make_pair(MOVEIT_TOTG, "TOTG时间最优轨迹"),  // 注释掉TOTG方法
            std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
            std::make_pair(B_SPLINE_3, "3阶B样条曲线"),
            std::make_pair(B_SPLINE_5, "5阶B样条曲线"),
            std::make_pair(NURBS, "NURBS曲线")
        }) {
            SmoothingMethod method = method_pair.first;
            std::string method_name = method_pair.second;

            RCLCPP_INFO(this->get_logger(), "\n开始测试平滑方法: %s", method_name.c_str());

            for (size_t i = 0; i < test_poses_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "测试第 %ld/50 个位姿", i + 1);

                // 设置目标位姿
                move_group_->setPoseTarget(test_poses_[i]);

                // 多解规划版本
                auto start_time = std::chrono::high_resolution_clock::now();
                std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
                const int MAX_PLANS = 5; // 生成5个规划方案

                // 存储最佳方案的索引和其总关节运动量
                int best_plan_index = -1;
                double min_joint_movement = std::numeric_limits<double>::max();

                // 多次尝试规划，收集成功的方案
                bool plan_success = false;
                for (int attempt = 0; attempt < MAX_PLANS; ++attempt) {
                    RCLCPP_INFO(this->get_logger(), "规划尝试 %d/%d", attempt + 1, MAX_PLANS);
                    
                    moveit::planning_interface::MoveGroupInterface::Plan current_plan;
                    auto plan_result = move_group_->plan(current_plan);
                    plan_success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
                    
                    if (plan_success) {
                        plans.push_back(current_plan);
                        
                        // 计算关节总运动量
                        double total_movement = calculateTotalJointMovement(current_plan);
                        
                        RCLCPP_INFO(this->get_logger(), "规划方案 %d 的总关节运动量: %.4f", attempt, total_movement);
                        
                        // 检查是否为最佳方案
                        if (total_movement < min_joint_movement) {
                            min_joint_movement = total_movement;
                            best_plan_index = plans.size() - 1;
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "规划尝试 %d 失败，错误代码: %d", attempt + 1, plan_result.val);
                    }
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                double planning_time = std::chrono::duration<double>(end_time - start_time).count();
                double average_planning_time = planning_time / MAX_PLANS;
                results_[method].planning_times.push_back(average_planning_time);

                // 使用最优规划方案
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (best_plan_index >= 0) {
                    RCLCPP_INFO(this->get_logger(), "选择方案 %d 作为最优解，总运动量: %.4f", 
                               best_plan_index, min_joint_movement);
                    
                    // 使用最优规划方案
                    plan = plans[best_plan_index];
                    plan_success = true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "未找到有效的规划方案");
                    plan_success = false;
                }

                if (plan_success)
                {
                    // 应用轨迹平滑
                    double smoothing_time = 0.0;
                    if (method != NONE)
                    {
                        start_time = std::chrono::high_resolution_clock::now();
                        
                        // 获取轨迹
                        robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
                        // rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

                        // 应用不同的平滑方法
                        bool smooth_success = false;
                        switch (method)
                        {
                        case MOVEIT_TOTG:
                            smooth_success = smoothTrajectoryWithTOTG(rt, plan.trajectory_);
                            break;
                        case CUBIC_POLYNOMIAL:
                            smooth_success = smoothTrajectoryWithCubicPolynomial(rt, plan.trajectory_);
                            break;
                        case B_SPLINE_3:
                            smooth_success = smoothTrajectoryWithBSpline_3(rt, plan.trajectory_);
                            break;
                        case B_SPLINE_5:
                            smooth_success = smoothTrajectoryWithBSpline_5(rt, plan.trajectory_);
                            break;
                        case NURBS:
                            smooth_success = smoothTrajectoryWithNURBS(rt, plan.trajectory_);
                            break;
                        default:
                            break;
                        }
                        
                        end_time = std::chrono::high_resolution_clock::now();
                        smoothing_time = std::chrono::duration<double>(end_time - start_time).count();
                        
                        if (smooth_success) {
                            // 更新计划中的轨迹
                            rt.getRobotTrajectoryMsg(plan.trajectory_);
                        } else {
                            RCLCPP_WARN(this->get_logger(), "轨迹平滑失败，使用原始轨迹");
                        }
                    }
                    results_[method].smoothing_times.push_back(smoothing_time);

                    // 评估路径质量
                    PathQualityMetrics metrics = evaluatePathQuality(plan);
                    results_[method].path_metrics.push_back(metrics);

                    RCLCPP_INFO(this->get_logger(), "规划成功，开始执行运动...");
                    // 执行运动
                    MoveItErrorCode exec_result = move_group_->execute(plan);

                    if (exec_result == MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "运动执行成功");
                        
                        // 获取轨迹执行时间并打印
                        double exec_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                                         plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec / 1e9;
                        RCLCPP_INFO(this->get_logger(), "实际执行时间: %.3f 秒", exec_time);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "运动执行失败，错误码: %i",
                                     static_cast<int>(exec_result.val));
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "路径规划失败");
                }

                move_group_->clearPoseTargets();

                // 添加短暂延时，让机器人稳定
                rclcpp::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    bool smoothTrajectoryWithTOTG(robot_trajectory::RobotTrajectory &trajectory, 
                                 const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }

            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();

            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);

            // 使用时间最优轨迹生成器进行平滑
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(trajectory, 1.0, 1.0); // 速度和加速度缩放因子

            RCLCPP_INFO(this->get_logger(), "使用TOTG平滑轨迹成功");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "TOTG轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    bool smoothTrajectoryWithCubicPolynomial(robot_trajectory::RobotTrajectory &trajectory, 
                                              const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene) {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }
            
            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();
            
            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);
            
            // 使用迭代时间参数化（基于三次多项式）
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(trajectory, 0.5, 0.5);

            RCLCPP_INFO(this->get_logger(), "使用三次多项式平滑轨迹成功");
            return true;
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "三次多项式轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    bool smoothTrajectoryWithBSpline(robot_trajectory::RobotTrajectory &trajectory,
                                     const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }

            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();

            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);

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

            // 对每个关节应用B样条平滑
            size_t num_joints = joint_positions[0].size();
            size_t num_points = joint_positions.size();

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());

            // 使用B样条插值生成更多的路径点
            size_t new_num_points = num_points * 2; // 增加采样点数量

            // 获取机器人当前状态的关节位置
            std::vector<double> current_joint_values;
            current_state.copyJointGroupPositions(move_group_->getName(), current_joint_values);

            for (size_t i = 0; i < new_num_points; ++i)
            {
                double t = i * time_stamps.back() / (new_num_points - 1);

                // 创建新的机器人状态
                moveit::core::RobotState new_state(trajectory.getRobotModel());

                // 初始化为当前状态
                new_state.setJointGroupPositions(move_group_->getName(), current_joint_values);

                std::vector<double> new_joint_values(num_joints);

                // 对每个关节应用B样条插值
                for (size_t j = 0; j < num_joints; ++j)
                {
                    // 收集该关节的所有位置
                    std::vector<double> joint_j_positions;
                    for (size_t k = 0; k < num_points; ++k)
                    {
                        joint_j_positions.push_back(joint_positions[k][j]);
                    }

                    // 特殊处理第一个点 - 使用当前状态
                    if (i == 0)
                    {
                        new_joint_values[j] = current_joint_values[j];
                    }
                    // 简单的线性插值（实际应用中应替换为B样条插值）
                    else if (t <= time_stamps.front())
                    {
                        // 在起始点附近，平滑过渡
                        double alpha = t / time_stamps.front();
                        new_joint_values[j] = current_joint_values[j] * (1 - alpha) + joint_j_positions.front() * alpha;
                    }
                    else if (t >= time_stamps.back())
                    {
                        new_joint_values[j] = joint_j_positions.back();
                    }
                    else
                    {
                        // 找到t所在的时间区间
                        size_t idx = 0;
                        while (idx < time_stamps.size() - 1 && t > time_stamps[idx + 1])
                        {
                            idx++;
                        }

                        double alpha = (t - time_stamps[idx]) / (time_stamps[idx + 1] - time_stamps[idx]);
                        new_joint_values[j] = joint_j_positions[idx] * (1 - alpha) + joint_j_positions[idx + 1] * alpha;
                    }
                }

                // 设置新的关节值
                new_state.setJointGroupPositions(move_group_->getName(), new_joint_values);

                // 添加到平滑轨迹
                smooth_trajectory.addSuffixWayPoint(new_state, t - (i > 0 ? time_stamps[i - 1] : 0));
            }

            // 更新原轨迹
            trajectory = smooth_trajectory;

            // 重新计算时间戳
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(trajectory, 0.5, 0.5);

            RCLCPP_INFO(this->get_logger(), "使用B样条曲线平滑轨迹成功");
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "B样条曲线轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    bool smoothTrajectoryWithBSpline_3(robot_trajectory::RobotTrajectory &trajectory,
                                     const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }

            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();

            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);

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

            // 对每个关节应用均匀有理B样条平滑
            size_t num_joints = joint_positions[0].size();
            size_t num_points = joint_positions.size();

            RCLCPP_INFO(this->get_logger(), "轨迹点数量: %ld", num_points);

            // 3阶B样条需要至少4个控制点
            const int k = 4; // 3阶B样条(degree=3, order=4)

            if (num_points < k)
            {
                RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，3阶B样条需要至少%d个点，使用原始轨迹",
                            num_points, k);
                return true;
            }

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());

            // 保留原始轨迹的第一个点
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(0), 0.0);

            // 新轨迹的总点数
            size_t new_num_points = std::max(size_t(20), num_points * 2);

            // 为所有关节创建均匀B样条节点向量
            Eigen::VectorXd knots(num_points + k);

            // Clamped节点向量（重复端点，确保曲线通过端点）
            for (int i = 0; i < k; ++i)
            {
                knots(i) = 0.0;
                knots(num_points + i - 1) = 1.0; // 注意：这里修复了可能的越界
            }

            // 中间节点均匀分布
            for (int i = 0; i < num_points - k + 1; ++i) // 修正循环边界条件
            {
                knots(k + i - 1) = static_cast<double>(i + 1) / (num_points - k + 2);
            }

            // 计算均匀时间间隔的新轨迹点
            for (size_t i = 1; i < new_num_points - 1; ++i)
            {
                // 归一化参数
                double u = static_cast<double>(i) / (new_num_points - 1);

                // 创建新状态
                moveit::core::RobotState new_state(trajectory.getRobotModel());
                new_state = trajectory.getWayPoint(0); // 复制第一个点的状态结构

                // 计算新的关节值
                std::vector<double> new_joint_values(num_joints, 0.0);

                // 对每个关节应用B样条插值
                for (size_t j = 0; j < num_joints; ++j)
                {
                    // 提取关节位置作为控制点
                    std::vector<double> control_points;
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        control_points.push_back(joint_positions[p][j]);
                    }

                    // 计算B样条值
                    double value = 0.0;
                    double denom = 0.0;

                    // 只处理有效的控制点索引范围
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        double basis = deBoorCoxBasisSafe(p, k - 1, u, knots);
                        value += basis * control_points[p];
                        denom += basis;
                    }

                    // 设置关节值
                    new_joint_values[j] = (denom > 1e-6) ? value / denom : control_points[0];
                }

                // 设置新状态的关节值
                new_state.setJointGroupPositions(move_group_->getName(), new_joint_values);

                // 计算时间，使用等间隔
                double total_time = time_stamps.back();
                double dt = total_time / (new_num_points - 1);

                // 添加到轨迹
                smooth_trajectory.addSuffixWayPoint(new_state, dt);
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
            if (!iptp.computeTimeStamps(trajectory, 0.5, 0.5))
            {
                RCLCPP_WARN(this->get_logger(), "时间参数化失败，使用简单时间分配");
            }

            RCLCPP_INFO(this->get_logger(), "使用3阶均匀B样条曲线平滑轨迹成功，点数: %ld",
                        trajectory.getWayPointCount());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "3阶B样条曲线轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    bool smoothTrajectoryWithBSpline_5(robot_trajectory::RobotTrajectory &trajectory, 
                                      const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }

            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();

            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);
            
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

            // 对每个关节应用均匀有理B样条平滑
            size_t num_joints = joint_positions[0].size();
            size_t num_points = joint_positions.size();

            RCLCPP_INFO(this->get_logger(), "轨迹点数量: %ld", num_points);
            
            // 5阶B样条需要至少6个控制点
            int k = 6; // 5阶B样条(degree=5, order=6)
            
            // 如果点数不足，退化为3阶B样条
            if (num_points < k) {
                k = 4; // 3阶B样条(degree=3, order=4)
                bspline5_degraded_count_++; // 记录退化次数
                RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，5阶B样条需要至少6个点，退化为3阶B样条(需要至少4个点)", 
                           num_points);
                
                // 检查是否满足3阶B样条的最小点数要求
                if (num_points < k) {
                    RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，3阶B样条需要至少%d个点，使用原始轨迹", 
                               num_points, k);
                    return true;
                }
            }

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());
            
            // 保留原始轨迹的第一个点
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(0), 0.0);
            
            // 新轨迹的总点数
            size_t new_num_points = std::max(size_t(20), num_points * 2);
            
            // 为所有关节创建均匀B样条节点向量
            Eigen::VectorXd knots(num_points + k);
            
            // Clamped节点向量（重复端点，确保曲线通过端点）
            for (int i = 0; i < k; ++i)
            {
                knots(i) = 0.0;
                knots(num_points + i - 1) = 1.0;  // 注意：这里修复了可能的越界
            }
            
            // 中间节点均匀分布
            for (int i = 0; i < num_points - k + 1; ++i)  // 修正循环边界条件
            {
                knots(k + i - 1) = static_cast<double>(i + 1) / (num_points - k + 2);
            }
            
            // 计算均匀时间间隔的新轨迹点
            for (size_t i = 1; i < new_num_points - 1; ++i)
            {
                // 归一化参数
                double u = static_cast<double>(i) / (new_num_points - 1);
                
                // 创建新状态
                moveit::core::RobotState new_state(trajectory.getRobotModel());
                new_state = trajectory.getWayPoint(0); // 复制第一个点的状态结构
                
                // 计算新的关节值
                std::vector<double> new_joint_values(num_joints, 0.0);
                
                // 对每个关节应用B样条插值
                for (size_t j = 0; j < num_joints; ++j)
                {
                    // 提取关节位置作为控制点
                    std::vector<double> control_points;
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        control_points.push_back(joint_positions[p][j]);
                    }
                    
                    // 计算B样条值
                    double value = 0.0;
                    double denom = 0.0;
                    
                    // 只处理有效的控制点索引范围
                    for (size_t p = 0; p < num_points; ++p)
                    {
                        double basis = deBoorCoxBasisSafe(p, k-1, u, knots);
                        value += basis * control_points[p];
                        denom += basis;
                    }
                    
                    // 设置关节值
                    new_joint_values[j] = (denom > 1e-6) ? value / denom : control_points[0];
                }
                
                // 设置新状态的关节值
                new_state.setJointGroupPositions(move_group_->getName(), new_joint_values);
                
                // 计算时间，使用等间隔
                double total_time = time_stamps.back();
                double dt = total_time / (new_num_points - 1);
                
                // 添加到轨迹
                smooth_trajectory.addSuffixWayPoint(new_state, dt);
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
            if (!iptp.computeTimeStamps(trajectory, 0.5, 0.5))
            {
                RCLCPP_WARN(this->get_logger(), "时间参数化失败，使用简单时间分配");
            }
            
            // 输出最终使用的B样条阶数和轨迹点数
            RCLCPP_INFO(this->get_logger(), "使用%d阶均匀B样条曲线平滑轨迹成功，点数: %ld", 
                       k-1, trajectory.getWayPointCount());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "B样条曲线轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    bool smoothTrajectoryWithNURBS(robot_trajectory::RobotTrajectory &trajectory, 
                                  const moveit_msgs::msg::RobotTrajectory &trajectory_msg)
    {
        try
        {
            // 使用规划场景中的机器人状态
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            if (!scene)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取规划场景");
                return false;
            }

            // 使用规划场景中的当前状态
            moveit::core::RobotState current_state = scene->getCurrentState();

            // 重新设置轨迹消息
            trajectory.clear();
            trajectory.setRobotTrajectoryMsg(current_state, trajectory_msg);
            
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

            RCLCPP_INFO(this->get_logger(), "轨迹点数量: %ld", num_points);
            
            // 5阶NURBS需要至少6个控制点
            int k = 6; // 5阶B样条(degree=5, order=6)
            
            // 如果点数不足，退化为3阶NURBS
            if (num_points < k) {
                k = 4; // 3阶B样条(degree=3, order=4)
                nurbs_degraded_count_++; // 记录退化次数
                RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，5阶NURBS需要至少6个点，退化为3阶NURBS(需要至少4个点)", 
                           num_points);
                
                // 检查是否满足3阶NURBS的最小点数要求
                if (num_points < k) {
                    RCLCPP_WARN(this->get_logger(), "轨迹点太少(%ld)，3阶NURBS需要至少%d个点，使用原始轨迹", 
                               num_points, k);
                    return true;
                }
            }

            // 创建新的平滑轨迹
            robot_trajectory::RobotTrajectory smooth_trajectory(trajectory.getRobotModel(), move_group_->getName());
            
            // 保留原始轨迹的第一个点
            smooth_trajectory.addSuffixWayPoint(trajectory.getWayPoint(0), 0.0);
            
            // 新轨迹的总点数
            size_t new_num_points = std::max(size_t(20), num_points * 2);
            
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
                    double diff = joint_positions[i][j] - joint_positions[i-1][j];
                    dist += diff * diff;
                }
                chord_lengths[i] = chord_lengths[i-1] + std::sqrt(dist);
            }
            
            // 归一化弦长
            double total_length = chord_lengths.back();
            if (total_length > 1e-8) // 避免除以零
            {
                for (size_t i = 0; i < num_points; ++i)
                {
                    chord_lengths[i] /= total_length;
                }
            }
            
            // 设置第一个和最后一个节点的重复度（Clamped B-spline曲线）
            for (int i = 0; i < k; ++i)
            {
                knots(i) = 0.0;
                knots(num_points + k - 1 - i) = 1.0;
            }
            
            // 根据弦长设置中间节点
            for (int i = 1; i < num_points - 1; ++i)
            {
                // 使用弦长参数化设置中间节点
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
                    // 对于位置p，曲率与二阶导数成正比: f''(x) ≈ (f(x+h) - 2f(x) + f(x-h))/h²
                    double prev = joint_positions[i-1][j];
                    double curr = joint_positions[i][j];
                    double next = joint_positions[i+1][j];
                    
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
                    
                    // 记录高曲率点位置
                    if (curvature_weight > 2.0) {
                        RCLCPP_INFO(this->get_logger(), "点 %ld 处检测到高曲率: %.3f，设置权重: %.2f", 
                                    i, max_curvature, curvature_weight);
                    }
                }
            }
            
            // 为起点和终点设置高权重，确保曲线精确通过
            weights(0) = 5.0;                 // 起点
            weights(num_points - 1) = 5.0;    // 终点
            weights(1) = std::max(weights(1), 2.0); // 第二点
            weights(num_points - 2) = std::max(weights(num_points - 2), 2.0); // 倒数第二点

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
                        double basis = nurbsBasisSafe(p, k-1, u, knots);
                        numerator += basis * weights(p) * control_points[p];
                        denominator += basis * weights(p);
                    }
                    
                    // 计算有理B样条值
                    if (denominator > 1e-8) // 避免除以零
                    {
                        new_joint_values[j] = numerator / denominator;
                    }
                    else
                    {
                        // 如果分母太小，使用插值控制点的起点
                        new_joint_values[j] = control_points[0];
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                           "NURBS分母近似为零，使用控制点起点值");
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
            if (!iptp.computeTimeStamps(trajectory, 0.5, 0.5))
            {
                RCLCPP_WARN(this->get_logger(), "时间参数化失败，使用简单时间分配");
            }
            
            // 输出最终使用的NURBS阶数和轨迹点数
            RCLCPP_INFO(this->get_logger(), "使用%d阶非均匀有理B样条(NURBS)曲线平滑轨迹成功，点数: %ld", 
                       k-1, trajectory.getWayPointCount());
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "NURBS曲线轨迹平滑失败: %s", e.what());
            return false;
        }
    }

    // 安全版本的De Boor-Cox B样条基函数
    double deBoorCoxBasisSafe(int i, int p, double u, const Eigen::VectorXd& knots)
    {
        // 越界检查
        if (i < 0 || i+p+1 >= knots.size()) {
            return 0.0;  // 越界返回0
        }
        
        // 0阶基函数
        if (p == 0)
        {
            // 确保索引在有效范围内
            if (i >= knots.size()-1) return 0.0;
            
            return (u >= knots(i) && u < knots(i+1)) || 
                   (u >= knots(i) && u <= knots(i+1) && i == knots.size()-2) ? 1.0 : 0.0;
        }
        
        // 递归计算前检查索引边界
        if (i+p >= knots.size() || i+1 >= knots.size() || i+p+1 >= knots.size()) {
            return 0.0;  // 避免递归中的索引越界
        }
        
        // 递归计算
        double coef1 = 0.0, coef2 = 0.0;
        
        if (knots(i+p) - knots(i) > 1e-10) // 避免除以零
        {
            coef1 = (u - knots(i)) / (knots(i+p) - knots(i));
        }
        
        if (knots(i+p+1) - knots(i+1) > 1e-10) // 避免除以零
        {
            coef2 = (knots(i+p+1) - u) / (knots(i+p+1) - knots(i+1));
        }
        
        // 递归调用同样使用安全版本
        return coef1 * deBoorCoxBasisSafe(i, p-1, u, knots) + 
               coef2 * deBoorCoxBasisSafe(i+1, p-1, u, knots);
    }

    // 计算非均匀有理B样条基函数（带安全检查）
    double nurbsBasisSafe(int i, int p, double u, const Eigen::VectorXd& knots)
    {
        // 越界检查
        if (i < 0 || i+p+1 >= knots.size()) {
            return 0.0;  // 越界返回0
        }
        
        // 0阶基函数
        if (p == 0)
        {
            // 确保索引在有效范围内
            if (i >= knots.size()-1) return 0.0;
            
            return (u >= knots(i) && u < knots(i+1)) || 
                   (u >= knots(i) && u <= knots(i+1) && i == knots.size()-2) ? 1.0 : 0.0;
        }
        
        // 递归计算前检查索引边界
        if (i+p >= knots.size() || i+1 >= knots.size() || i+p+1 >= knots.size()) {
            return 0.0;  // 避免递归中的索引越界
        }
        
        // 递归计算
        double coef1 = 0.0, coef2 = 0.0;
        
        if (knots(i+p) - knots(i) > 1e-10) // 避免除以零
        {
            coef1 = (u - knots(i)) / (knots(i+p) - knots(i));
        }
        
        if (knots(i+p+1) - knots(i+1) > 1e-10) // 避免除以零
        {
            coef2 = (knots(i+p+1) - u) / (knots(i+p+1) - knots(i+1));
        }
        
        // 递归调用同样使用安全版本
        return coef1 * nurbsBasisSafe(i, p-1, u, knots) + 
               coef2 * nurbsBasisSafe(i+1, p-1, u, knots);
    }

    // 根据弦长参数化计算非均匀参数u对应的实际参数
    double evaluateNonUniformParameter(double u, const std::vector<double>& chord_lengths)
    {
        if (chord_lengths.empty()) return u;
        
        // 搜索对应区间
        size_t idx = 0;
        while (idx < chord_lengths.size() - 1 && u > chord_lengths[idx+1])
        {
            idx++;
        }
        
        // 如果u超出范围，返回边界值
        if (idx >= chord_lengths.size() - 1) {
            return chord_lengths.back();
        }
        
        // 线性插值
        if (idx < chord_lengths.size() - 1)
        {
            double t0 = chord_lengths[idx];
            double t1 = chord_lengths[idx+1];
            
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

    PathQualityMetrics evaluatePathQuality(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        PathQualityMetrics metrics;

        metrics.path_length = calculatePathLength(plan.trajectory_);
        metrics.joint_movement = calculateJointMovement(plan.trajectory_);
        metrics.smoothness = calculateSmoothness(plan.trajectory_);
        metrics.waypoint_count = plan.trajectory_.joint_trajectory.points.size();
        metrics.execution_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                                 plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec / 1e9;

        // 输出当前路径的质量指标
        RCLCPP_INFO(this->get_logger(), "路径质量评估结果:");
        RCLCPP_INFO(this->get_logger(), "- 路径长度: %.3f 米", metrics.path_length);
        RCLCPP_INFO(this->get_logger(), "- 关节总移动量: %.3f 弧度", metrics.joint_movement);
        RCLCPP_INFO(this->get_logger(), "- 路径平滑度: %.3f", metrics.smoothness);
        RCLCPP_INFO(this->get_logger(), "- 路径点数量: %d", metrics.waypoint_count);
        RCLCPP_INFO(this->get_logger(), "- 执行时间: %.3f 秒", metrics.execution_time);

        return metrics;
    }

    void saveResults(int run_index)
    {
        // 构造文件名，使用相对路径
        std::stringstream ss;
        ss << "src/planning_node/test_results/smoothing_test_results_" << run_index << ".csv";

        // 创建目录（如果不存在）
        std::filesystem::path dir_path("src/planning_node/test_results");
        if (!std::filesystem::exists(dir_path))
        {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "创建目录: %s",
                        std::filesystem::absolute(dir_path).string().c_str());
        }

        std::ofstream csv_file(ss.str());
        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", ss.str().c_str());
            return;
        }

        // 输出实际保存路径
        RCLCPP_INFO(this->get_logger(), "结果保存在: %s",
                    std::filesystem::absolute(ss.str()).string().c_str());

        // 保存详细数据到CSV文件
        csv_file << "Position,Method,Planning Time (s),Smoothing Time (s),Total Time (s),Path Length (m),Joint Movement (rad),Smoothness,Waypoint Count,Execution Time (s)\n";

        // 遍历所有测试点和平滑方法
        for (size_t i = 0; i < test_poses_.size(); i++)
        {
            for (const auto &method_pair : {
                     std::make_pair(NONE, "线性插值"),
                     std::make_pair(MOVEIT_TOTG, "TOTG时间最优轨迹"),  // 注释掉TOTG方法
                     std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                     std::make_pair(B_SPLINE_3, "3阶B样条曲线"),
                     std::make_pair(B_SPLINE_5, "5阶B样条曲线"),
                     std::make_pair(NURBS, "NURBS曲线")})
            {
                SmoothingMethod method = method_pair.first;
                std::string method_name = method_pair.second;

                // 确保有足够的数据
                if (i < results_[method].planning_times.size() &&
                    i < results_[method].smoothing_times.size() &&
                    i < results_[method].path_metrics.size())
                {
                    double planning_time = results_[method].planning_times[i];
                    double smoothing_time = results_[method].smoothing_times[i];
                    double total_time = planning_time + smoothing_time; // 计算总时间
                    const auto &metrics = results_[method].path_metrics[i];

                    csv_file << i << ","
                             << method_name << ","
                             << planning_time << ","
                             << smoothing_time << ","
                             << total_time << "," // 添加总时间列
                             << metrics.path_length << ","
                             << metrics.joint_movement << ","
                             << metrics.smoothness << ","
                             << metrics.waypoint_count << ","
                             << metrics.execution_time << "\n";
                }
            }
        }

        // 计算平均值
        std::map<SmoothingMethod, double> avg_planning_times;
        std::map<SmoothingMethod, double> avg_smoothing_times;
        std::map<SmoothingMethod, double> avg_total_times; // 添加总时间平均值
        std::map<SmoothingMethod, double> avg_path_lengths;
        std::map<SmoothingMethod, double> avg_joint_movements;
        std::map<SmoothingMethod, double> avg_smoothness;
        std::map<SmoothingMethod, double> avg_waypoint_counts;
        std::map<SmoothingMethod, double> avg_execution_times;

        for (const auto &method_pair : {
                 std::make_pair(NONE, "线性插值"),
                 std::make_pair(MOVEIT_TOTG, "TOTG时间最优轨迹"),  // 注释掉TOTG方法
                 std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                 std::make_pair(B_SPLINE_3, "3阶B样条曲线"),
                 std::make_pair(B_SPLINE_5, "5阶B样条曲线"),
                 std::make_pair(NURBS, "NURBS曲线")})
        {
            SmoothingMethod method = method_pair.first;
            
            // 计算平均值
            if (!results_[method].planning_times.empty())
            {
                avg_planning_times[method] = std::accumulate(
                    results_[method].planning_times.begin(),
                    results_[method].planning_times.end(), 0.0) /
                                            results_[method].planning_times.size();
            }
            
            if (!results_[method].smoothing_times.empty())
            {
                avg_smoothing_times[method] = std::accumulate(
                    results_[method].smoothing_times.begin(),
                    results_[method].smoothing_times.end(), 0.0) /
                                             results_[method].smoothing_times.size();
            }
            
            // 计算总时间平均值
            avg_total_times[method] = avg_planning_times[method] + avg_smoothing_times[method];
            
            if (!results_[method].path_metrics.empty())
            {
                double sum_path_length = 0.0;
                double sum_joint_movement = 0.0;
                double sum_smoothness = 0.0;
                double sum_waypoint_count = 0.0;
                double sum_execution_time = 0.0;
                
                for (const auto &metrics : results_[method].path_metrics)
                {
                    sum_path_length += metrics.path_length;
                    sum_joint_movement += metrics.joint_movement;
                    sum_smoothness += metrics.smoothness;
                    sum_waypoint_count += metrics.waypoint_count;
                    sum_execution_time += metrics.execution_time;
                }
                
                size_t count = results_[method].path_metrics.size();
                avg_path_lengths[method] = sum_path_length / count;
                avg_joint_movements[method] = sum_joint_movement / count;
                avg_smoothness[method] = sum_smoothness / count;
                avg_waypoint_counts[method] = sum_waypoint_count / count;
                avg_execution_times[method] = sum_execution_time / count;
            }
        }

        // 保存平均值
        csv_file << "\nStatistics:\n";
        csv_file << ",Method,Planning Time (s),Smoothing Time (s),Total Time (s),Path Length (m),Joint Movement (rad),Smoothness,Waypoint Count,Execution Time (s)\n";
        
        for (const auto &method_pair : {
                 std::make_pair(NONE, "线性插值"),
                 std::make_pair(MOVEIT_TOTG, "TOTG时间最优轨迹"),  // 注释掉TOTG方法
                 std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                 std::make_pair(B_SPLINE_3, "3阶B样条曲线"),
                 std::make_pair(B_SPLINE_5, "5阶B样条曲线"),
                 std::make_pair(NURBS, "NURBS曲线")})
        {
            SmoothingMethod method = method_pair.first;
            std::string method_name = method_pair.second;
            
            csv_file << ","
                     << method_name << ","
                     << avg_planning_times[method] << ","
                     << avg_smoothing_times[method] << ","
                     << avg_total_times[method] << "," // 添加总时间平均值
                     << avg_path_lengths[method] << ","
                     << avg_joint_movements[method] << ","
                     << avg_smoothness[method] << ","
                     << avg_waypoint_counts[method] << ","
                     << avg_execution_times[method] << "\n";
        }

        // 在保存平均值部分之后，添加退化计数统计
        csv_file << "\nDegradation Statistics:\n";
        csv_file << "Method,Degraded Count\n";
        csv_file << "5阶B样条曲线," << bspline5_degraded_count_ << "\n";
        csv_file << "NURBS曲线," << nurbs_degraded_count_ << "\n";
        
        csv_file.close();

        // 保存绘图数据
        saveChartData(run_index, avg_planning_times, avg_smoothing_times, avg_total_times, 
                     avg_path_lengths, avg_joint_movements, avg_smoothness, avg_waypoint_counts,
                     avg_execution_times);
    }

    void saveChartData(int run_index, 
                      const std::map<SmoothingMethod, double>& avg_planning_times,
                      const std::map<SmoothingMethod, double>& avg_smoothing_times,
                      const std::map<SmoothingMethod, double>& avg_total_times,
                      const std::map<SmoothingMethod, double>& avg_path_lengths,
                      const std::map<SmoothingMethod, double>& avg_joint_movements,
                      const std::map<SmoothingMethod, double>& avg_smoothness,
                      const std::map<SmoothingMethod, double>& avg_waypoint_counts,
                      const std::map<SmoothingMethod, double>& avg_execution_times)
    {
        // 构造文件名
        std::stringstream ss;
        ss << "src/planning_node/test_results/smoothing_chart_data_" << run_index << ".csv";
        
        std::ofstream chart_file(ss.str());
        if (!chart_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建绘图数据文件: %s", ss.str().c_str());
            return;
        }
        
        // 保存平均值数据，便于绘制柱状图
        chart_file << "Metric,线性插值,TOTG时间最优轨迹,三次多项式,3阶B样条曲线,5阶B样条曲线,NURBS\n";
        
        // 写入每个指标的平均值
        chart_file << "规划时间(s)," 
                  << avg_planning_times.at(NONE) << "," 
                  << avg_planning_times.at(MOVEIT_TOTG) << "," 
                  << avg_planning_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_planning_times.at(B_SPLINE_3) << "," 
                  << avg_planning_times.at(B_SPLINE_5) << "," 
                  << avg_planning_times.at(NURBS) << "\n";
                  
        chart_file << "平滑时间(s)," 
                  << avg_smoothing_times.at(NONE) << "," 
                  << avg_smoothing_times.at(MOVEIT_TOTG) << "," 
                  << avg_smoothing_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_smoothing_times.at(B_SPLINE_3) << "," 
                  << avg_smoothing_times.at(B_SPLINE_5) << "," 
                  << avg_smoothing_times.at(NURBS) << "\n";
                  
        chart_file << "总时间(s)," 
                  << avg_total_times.at(NONE) << "," 
                  << avg_total_times.at(MOVEIT_TOTG) << "," 
                  << avg_total_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_total_times.at(B_SPLINE_3) << "," 
                  << avg_total_times.at(B_SPLINE_5) << "," 
                  << avg_total_times.at(NURBS) << "\n";
                  
        chart_file << "路径长度(m)," 
                  << avg_path_lengths.at(NONE) << "," 
                  << avg_path_lengths.at(MOVEIT_TOTG) << "," 
                  << avg_path_lengths.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_path_lengths.at(B_SPLINE_3) << "," 
                  << avg_path_lengths.at(B_SPLINE_5) << "," 
                  << avg_path_lengths.at(NURBS) << "\n";
                  
        chart_file << "关节移动量(rad)," 
                  << avg_joint_movements.at(NONE) << "," 
                  << avg_joint_movements.at(MOVEIT_TOTG) << "," 
                  << avg_joint_movements.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_joint_movements.at(B_SPLINE_3) << "," 
                  << avg_joint_movements.at(B_SPLINE_5) << "," 
                  << avg_joint_movements.at(NURBS) << "\n";
                  
        chart_file << "平滑度," 
                  << avg_smoothness.at(NONE) << "," 
                  << avg_smoothness.at(MOVEIT_TOTG) << "," 
                  << avg_smoothness.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_smoothness.at(B_SPLINE_3) << "," 
                  << avg_smoothness.at(B_SPLINE_5) << "," 
                  << avg_smoothness.at(NURBS) << "\n";
                  
        chart_file << "路径点数量," 
                  << avg_waypoint_counts.at(NONE) << "," 
                  << avg_waypoint_counts.at(MOVEIT_TOTG) << "," 
                  << avg_waypoint_counts.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_waypoint_counts.at(B_SPLINE_3) << "," 
                  << avg_waypoint_counts.at(B_SPLINE_5) << "," 
                  << avg_waypoint_counts.at(NURBS) << "\n";
                  
        chart_file << "执行时间(s)," 
                  << avg_execution_times.at(NONE) << "," 
                  << avg_execution_times.at(MOVEIT_TOTG) << "," 
                  << avg_execution_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_execution_times.at(B_SPLINE_3) << "," 
                  << avg_execution_times.at(B_SPLINE_5) << "," 
                  << avg_execution_times.at(NURBS) << "\n";
        
        // 在末尾添加退化统计信息
        chart_file << "\n退化统计:\n";
        chart_file << "方法,退化点位数\n";
        chart_file << "5阶B样条曲线," << bspline5_degraded_count_ << "\n";
        chart_file << "NURBS曲线," << nurbs_degraded_count_ << "\n";

        
        chart_file.close();
        
        RCLCPP_INFO(this->get_logger(), "绘图数据保存在: %s",
                    std::filesystem::absolute(ss.str()).string().c_str());
        
        // 打印退化统计
        RCLCPP_INFO(this->get_logger(), "B样条退化点位统计: 5阶B样条退化次数: %d, NURBS退化次数: %d",
                   bspline5_degraded_count_, nurbs_degraded_count_);
    }

    void clearTestData()
    {
        for (auto &result_pair : results_)
        {
            result_pair.second.planning_times.clear();
            result_pair.second.smoothing_times.clear();
            result_pair.second.path_metrics.clear();
        }
        
        // 清除退化计数器
        bspline5_degraded_count_ = 0;
        nurbs_degraded_count_ = 0;
    }

    void printPlannerInfo()
    {
        RCLCPP_INFO(this->get_logger(), "\n当前规划器配置信息：");
        
        // 获取当前规划器ID
        std::string planner_id = move_group_->getPlanningPipelineId();
        RCLCPP_INFO(this->get_logger(), "当前使用的规划器ID: %s", 
            planner_id.empty() ? "默认规划器" : planner_id.c_str());

        // 获取规划组名称
        std::string planning_group = move_group_->getPlanningFrame();
        RCLCPP_INFO(this->get_logger(), "当前规划组参考坐标系: %s", planning_group.c_str());

        // 获取末端执行器链接
        std::string end_effector = move_group_->getEndEffectorLink();
        RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", end_effector.c_str());

        // 获取当前的运动学参数
        double goal_position_tolerance = move_group_->getGoalPositionTolerance();
        double goal_orientation_tolerance = move_group_->getGoalOrientationTolerance();
        double planning_time = move_group_->getPlanningTime();

        RCLCPP_INFO(this->get_logger(), "当前规划参数:");
        RCLCPP_INFO(this->get_logger(), "- 位置容差: %.3f 米", goal_position_tolerance);
        RCLCPP_INFO(this->get_logger(), "- 姿态容差: %.3f 弧度", goal_orientation_tolerance);
        RCLCPP_INFO(this->get_logger(), "- 规划时间: %.1f 秒", planning_time);
    }

    void setPlannerConfig() {
        // 设置规划器
        move_group_->setPlannerId("BiTRRT");  // 使用BiTRRT规划器

        // 设置规划时间
        move_group_->setPlanningTime(10.0);  // 单位：秒

        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        
        // 设置规划尝试次数
        move_group_->setNumPlanningAttempts(10);
        
        // 设置目标位置容差
        move_group_->setGoalPositionTolerance(0.01);  // 单位：米
        move_group_->setGoalOrientationTolerance(0.01);  // 单位：弧度
    }

    double calculatePathLength(const moveit_msgs::msg::RobotTrajectory& trajectory) {
        double path_length = 0.0;
        const auto& points = trajectory.joint_trajectory.points;
        
        if (points.size() < 2) return 0.0;

        for (size_t i = 1; i < points.size(); ++i) {
            // 获取相邻两个点的位姿
            geometry_msgs::msg::Pose pose1, pose2;
            moveit::core::RobotState state1(move_group_->getRobotModel());
            moveit::core::RobotState state2(move_group_->getRobotModel());
            
            state1.setJointGroupPositions(move_group_->getName(), points[i-1].positions);
            state2.setJointGroupPositions(move_group_->getName(), points[i].positions);
            
            const Eigen::Isometry3d& transform1 = state1.getGlobalLinkTransform(move_group_->getEndEffectorLink());
            const Eigen::Isometry3d& transform2 = state2.getGlobalLinkTransform(move_group_->getEndEffectorLink());
            
            // 计算欧氏距离
            double dx = transform2.translation().x() - transform1.translation().x();
            double dy = transform2.translation().y() - transform1.translation().y();
            double dz = transform2.translation().z() - transform1.translation().z();
            path_length += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        return path_length;
    }

    double calculateJointMovement(const moveit_msgs::msg::RobotTrajectory& trajectory) {
        double total_movement = 0.0;
        const auto& points = trajectory.joint_trajectory.points;
        
        if (points.size() < 2) return 0.0;

        for (size_t i = 1; i < points.size(); ++i) {
            for (size_t j = 0; j < points[i].positions.size(); ++j) {
                double delta = points[i].positions[j] - points[i-1].positions[j];
                total_movement += std::abs(delta);
            }
        }
        return total_movement;
    }

    double calculateSmoothness(const moveit_msgs::msg::RobotTrajectory& trajectory) {
        double smoothness = 0.0;
        const auto& points = trajectory.joint_trajectory.points;
        
        if (points.size() < 3) return 0.0;

        for (size_t i = 1; i < points.size() - 1; ++i) {
            for (size_t j = 0; j < points[i].velocities.size(); ++j) {
                double vel_diff = points[i+1].velocities[j] - points[i-1].velocities[j];
                smoothness += vel_diff * vel_diff;
            }
        }
        return -std::sqrt(smoothness); // 负值表示不平滑度
    }

    // 计算轨迹的总关节运动量
    double calculateTotalJointMovement(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        double total_movement = 0.0;
        
        if (plan.trajectory_.joint_trajectory.points.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "轨迹点数量不足，无法计算关节运动量");
            return 0.0;
        }
        
        const auto& first_point = plan.trajectory_.joint_trajectory.points.front();
        const auto& last_point = plan.trajectory_.joint_trajectory.points.back();
        
        // 计算起点到终点的总关节运动量
        for (size_t j = 0; j < first_point.positions.size(); j++) {
            // 计算关节起点和终点的差值
            double joint_diff = std::abs(last_point.positions[j] - first_point.positions[j]);
            total_movement += joint_diff;
        }
        
        return total_movement;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectorySmoothingTest>("trajectory_smoothing_test");
    
    try {
        // 先初始化节点
        node->initialize();
        
        // 执行5次测试
        for (int i = 1; i <= 5; i++) {
            RCLCPP_INFO(node->get_logger(), "\n开始第 %d 次测试...", i);
            
            // 运行测试
            node->runTest();
            
            // 保存结果
            node->saveResults(i);
            
            // 清除数据，准备下一次测试
            node->clearTestData();
            
            // 如果不是最后一次测试，等待一段时间再开始下一次
            if (i < 5) {
                RCLCPP_INFO(node->get_logger(), "等待5秒后开始下一次测试...");
                rclcpp::sleep_for(std::chrono::seconds(5));
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "发生错误: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}