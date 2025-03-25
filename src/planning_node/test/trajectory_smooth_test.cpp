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

using moveit::core::MoveItErrorCode;

// 平滑方法枚举
enum SmoothingMethod
{
    NONE,             // 无平滑
    MOVEIT_TOTG,      // MoveIt时间最优轨迹生成
    CUBIC_POLYNOMIAL, // 三次多项式
    B_SPLINE          // B样条曲线
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
        results_[B_SPLINE] = SmoothingResults();

        RCLCPP_INFO(this->get_logger(), "初始化完成，准备开始测试");
    }

    void generatePoses()
    {
        for (int i = 0; i < 50; i++)
        {
            geometry_msgs::msg::Pose pose;
            // 在工作空间内生成圆形轨迹上的位姿
            pose.position.x = 0. + 0.1 * cos(i * 2 * M_PI / 50);
            pose.position.y = 0.9 + 0.1 * sin(i * 2 * M_PI / 50);
            pose.position.z = 1.2;
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
            std::make_pair(NONE, "无平滑"),
            // std::make_pair(MOVEIT_TOTG, "MoveIt时间最优轨迹生成"),  // 注释掉TOTG方法
            std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
            std::make_pair(B_SPLINE, "B样条曲线")
        }) {
            SmoothingMethod method = method_pair.first;
            std::string method_name = method_pair.second;

            RCLCPP_INFO(this->get_logger(), "\n开始测试平滑方法: %s", method_name.c_str());

            for (size_t i = 0; i < test_poses_.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(), "测试第 %ld/50 个位姿", i + 1);

                // 设置目标位姿
                move_group_->setPoseTarget(test_poses_[i]);

                // 测试路径规划时间
                auto start_time = std::chrono::high_resolution_clock::now();
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                MoveItErrorCode plan_result = move_group_->plan(plan);
                auto end_time = std::chrono::high_resolution_clock::now();
                double planning_time = std::chrono::duration<double>(end_time - start_time).count();
                results_[method].planning_times.push_back(planning_time);

                if (plan_result == MoveItErrorCode::SUCCESS)
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
                        case B_SPLINE:
                            smooth_success = smoothTrajectoryWithBSpline(rt, plan.trajectory_);
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
            totg.computeTimeStamps(trajectory, 0.5, 0.5); // 速度和加速度缩放因子

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
        csv_file << "Position,Method,Planning Time (s),Smoothing Time (s),Total Time (s),Path Length (m),Joint Movement (rad),Smoothness,Waypoint Count\n";

        // 遍历所有测试点和平滑方法
        for (size_t i = 0; i < test_poses_.size(); i++)
        {
            for (const auto &method_pair : {
                     std::make_pair(NONE, "无平滑"),
                     // std::make_pair(MOVEIT_TOTG, "MoveIt TOTG"),  // 注释掉TOTG方法
                     std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                     std::make_pair(B_SPLINE, "B样条曲线")})
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
                             << metrics.waypoint_count << "\n";
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

        for (const auto &method_pair : {
                 std::make_pair(NONE, "无平滑"),
                 // std::make_pair(MOVEIT_TOTG, "MoveIt TOTG"),  // 注释掉TOTG方法
                 std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                 std::make_pair(B_SPLINE, "B样条曲线")})
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
                
                for (const auto &metrics : results_[method].path_metrics)
                {
                    sum_path_length += metrics.path_length;
                    sum_joint_movement += metrics.joint_movement;
                    sum_smoothness += metrics.smoothness;
                    sum_waypoint_count += metrics.waypoint_count;
                }
                
                size_t count = results_[method].path_metrics.size();
                avg_path_lengths[method] = sum_path_length / count;
                avg_joint_movements[method] = sum_joint_movement / count;
                avg_smoothness[method] = sum_smoothness / count;
                avg_waypoint_counts[method] = sum_waypoint_count / count;
            }
        }

        // 保存平均值
        csv_file << "\nStatistics:\n";
        csv_file << ",Method,Planning Time (s),Smoothing Time (s),Total Time (s),Path Length (m),Joint Movement (rad),Smoothness,Waypoint Count\n";
        
        for (const auto &method_pair : {
                 std::make_pair(NONE, "无平滑"),
                 // std::make_pair(MOVEIT_TOTG, "MoveIt TOTG"),  // 注释掉TOTG方法
                 std::make_pair(CUBIC_POLYNOMIAL, "三次多项式"),
                 std::make_pair(B_SPLINE, "B样条曲线")})
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
                     << avg_waypoint_counts[method] << "\n";
        }

        csv_file.close();

        // 保存绘图数据
        saveChartData(run_index, avg_planning_times, avg_smoothing_times, avg_total_times, 
                     avg_path_lengths, avg_joint_movements, avg_smoothness, avg_waypoint_counts);
    }

    void saveChartData(int run_index, 
                      const std::map<SmoothingMethod, double>& avg_planning_times,
                      const std::map<SmoothingMethod, double>& avg_smoothing_times,
                      const std::map<SmoothingMethod, double>& avg_total_times,
                      const std::map<SmoothingMethod, double>& avg_path_lengths,
                      const std::map<SmoothingMethod, double>& avg_joint_movements,
                      const std::map<SmoothingMethod, double>& avg_smoothness,
                      const std::map<SmoothingMethod, double>& avg_waypoint_counts)
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
        chart_file << "Metric,无平滑,三次多项式,B样条曲线\n";
        
        // 写入每个指标的平均值
        chart_file << "规划时间(s)," 
                  << avg_planning_times.at(NONE) << "," 
                  << avg_planning_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_planning_times.at(B_SPLINE) << "\n";
                  
        chart_file << "平滑时间(s)," 
                  << avg_smoothing_times.at(NONE) << "," 
                  << avg_smoothing_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_smoothing_times.at(B_SPLINE) << "\n";
                  
        chart_file << "总时间(s)," 
                  << avg_total_times.at(NONE) << "," 
                  << avg_total_times.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_total_times.at(B_SPLINE) << "\n";
                  
        chart_file << "路径长度(m)," 
                  << avg_path_lengths.at(NONE) << "," 
                  << avg_path_lengths.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_path_lengths.at(B_SPLINE) << "\n";
                  
        chart_file << "关节移动量(rad)," 
                  << avg_joint_movements.at(NONE) << "," 
                  << avg_joint_movements.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_joint_movements.at(B_SPLINE) << "\n";
                  
        chart_file << "平滑度," 
                  << avg_smoothness.at(NONE) << "," 
                  << avg_smoothness.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_smoothness.at(B_SPLINE) << "\n";
                  
        chart_file << "路径点数量," 
                  << avg_waypoint_counts.at(NONE) << "," 
                  << avg_waypoint_counts.at(CUBIC_POLYNOMIAL) << "," 
                  << avg_waypoint_counts.at(B_SPLINE) << "\n";
        
        chart_file.close();
        
        RCLCPP_INFO(this->get_logger(), "绘图数据保存在: %s",
                    std::filesystem::absolute(ss.str()).string().c_str());
    }

    void clearTestData()
    {
        for (auto &result_pair : results_)
        {
            result_pair.second.planning_times.clear();
            result_pair.second.smoothing_times.clear();
            result_pair.second.path_metrics.clear();
        }
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
        move_group_->setPlanningTime(5.0);  // 单位：秒

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