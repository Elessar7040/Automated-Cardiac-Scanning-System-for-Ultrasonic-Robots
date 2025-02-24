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

using moveit::core::MoveItErrorCode;  // 使用新的错误码类型

class IKPlanningTest : public rclcpp::Node {
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<geometry_msgs::msg::Pose> test_poses_;
    std::vector<double> ik_times_;
    std::vector<double> planning_times_;
    struct PathQualityMetrics {
        double path_length;          // 笛卡尔空间路径长度
        double joint_movement;       // 关节空间总移动量
        double smoothness;           // 路径平滑度
        int waypoint_count;         // 路径点数量
        double min_clearance;       // 最小避障距离
    };
    std::vector<PathQualityMetrics> path_metrics_;
    
public:
    IKPlanningTest(const std::string& node_name)
        : Node(node_name) {
    }

    void initialize() {
        // 等待必要的服务启动
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // 初始化MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "russ_group");  // 根据实际机器人修改组名
        
        setPlannerConfig();

        // 打印规划器信息
        printPlannerInfo();
        
        generatePoses();
    }

    void generatePoses() {
        for (int i = 0; i < 50; i++) {
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

    void runTest() {
        for (size_t i = 0; i < test_poses_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "测试第 %ld/50 个位姿", i + 1);

            // 测试运动学解算时间
            auto start_time = std::chrono::high_resolution_clock::now();
            move_group_->setPoseTarget(test_poses_[i]);
            auto end_time = std::chrono::high_resolution_clock::now();
            double ik_time = std::chrono::duration<double>(end_time - start_time).count();
            ik_times_.push_back(ik_time);

            // 测试路径规划时间
            start_time = std::chrono::high_resolution_clock::now();
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            MoveItErrorCode plan_result = move_group_->plan(plan);
            end_time = std::chrono::high_resolution_clock::now();
            double planning_time = std::chrono::duration<double>(end_time - start_time).count();
            planning_times_.push_back(planning_time);

            if (plan_result == MoveItErrorCode::SUCCESS) {
                // 评估路径质量
                evaluatePathQuality(plan);
                
                RCLCPP_INFO(this->get_logger(), "规划成功，开始执行运动...");
                // 执行运动
                MoveItErrorCode exec_result = move_group_->execute(plan);
                
                if (exec_result == MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "运动执行成功");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "运动执行失败，错误码: %i", 
                        static_cast<int>(exec_result.val));
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "路径规划失败");
            }

            move_group_->clearPoseTargets();
            
            // 添加短暂延时，让机器人稳定
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void saveResults(int run_index) {
        // 计算统计数据
        double total_ik_time = 0.0;
        double total_planning_time = 0.0;
        for (size_t i = 0; i < ik_times_.size(); i++) {
            total_ik_time += ik_times_[i];
            total_planning_time += planning_times_[i];
        }
        double avg_ik_time = total_ik_time / ik_times_.size();
        double avg_planning_time = total_planning_time / planning_times_.size();

        // 计算平均质量指标
        double avg_path_length = 0.0;
        double avg_joint_movement = 0.0;
        double avg_smoothness = 0.0;
        double avg_waypoint_count = 0.0;
        
        for (const auto& metrics : path_metrics_) {
            avg_path_length += metrics.path_length;
            avg_joint_movement += metrics.joint_movement;
            avg_smoothness += metrics.smoothness;
            avg_waypoint_count += metrics.waypoint_count;
        }
        
        size_t count = path_metrics_.size();
        avg_path_length /= count;
        avg_joint_movement /= count;
        avg_smoothness /= count;
        avg_waypoint_count /= count;

        // 构造文件名，使用相对路径
        std::stringstream ss;
        ss << "src/planning_node/test_results/ik_planning_test_results_" << run_index << ".csv";
        
        // 创建目录（如果不存在）
        std::filesystem::path dir_path("src/planning_node/test_results");
        if (!std::filesystem::exists(dir_path)) {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(this->get_logger(), "创建目录: %s", 
                std::filesystem::absolute(dir_path).string().c_str());
        }
        
        std::ofstream csv_file(ss.str());
        if (!csv_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", ss.str().c_str());
            return;
        }

        // 输出实际保存路径
        RCLCPP_INFO(this->get_logger(), "结果保存在: %s", 
            std::filesystem::absolute(ss.str()).string().c_str());

        // 保存详细数据到CSV文件
        csv_file << "Position,IK Time (s),Planning Time (s),Path Length (m),Joint Movement (rad),Smoothness,Waypoint Count\n";
        
        for (size_t i = 0; i < ik_times_.size(); i++) {
            csv_file << i + 1 << "," 
                    << std::fixed << std::setprecision(6) << ik_times_[i] << "," 
                    << planning_times_[i] << ","
                    << path_metrics_[i].path_length << ","
                    << path_metrics_[i].joint_movement << ","
                    << path_metrics_[i].smoothness << ","
                    << path_metrics_[i].waypoint_count << "\n";
        }
        
        // 添加统计数据
        csv_file << "\nStatistics:\n";
        csv_file << "Metric,Average Value\n";
        csv_file << "IK Time (s)," << avg_ik_time << "\n";
        csv_file << "Planning Time (s)," << avg_planning_time << "\n";
        csv_file << "Path Length (m)," << avg_path_length << "\n";
        csv_file << "Joint Movement (rad)," << avg_joint_movement << "\n";
        csv_file << "Smoothness," << avg_smoothness << "\n";
        csv_file << "Waypoint Count," << avg_waypoint_count << "\n";
        
        csv_file.close();

        // 在终端显示统计结果
        RCLCPP_INFO(this->get_logger(), "\n第 %d 次测试结果统计：", run_index);
        RCLCPP_INFO(this->get_logger(), "时间统计:");
        RCLCPP_INFO(this->get_logger(), "- 运动学解算平均时间: %.6f 秒", avg_ik_time);
        RCLCPP_INFO(this->get_logger(), "- 路径规划平均时间: %.6f 秒", avg_planning_time);
        RCLCPP_INFO(this->get_logger(), "路径质量统计:");
        RCLCPP_INFO(this->get_logger(), "- 路径长度平均值: %.6f 米", avg_path_length);
        RCLCPP_INFO(this->get_logger(), "- 关节移动量平均值: %.6f 弧度", avg_joint_movement);
        RCLCPP_INFO(this->get_logger(), "- 路径平滑度平均值: %.6f", avg_smoothness);
        RCLCPP_INFO(this->get_logger(), "- 路径点数量平均值: %.1f", avg_waypoint_count);
    }

    void clearTestData() {
        ik_times_.clear();
        planning_times_.clear();
        path_metrics_.clear();
    }

    void printPlannerInfo() {
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
        move_group_->setPlannerId("RRTConnect");  // 或其他可用的规划器ID
        // move_group_->setPlannerId("BiTRRT");  // 或其他可用的规划器ID

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

    void evaluatePathQuality(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        PathQualityMetrics metrics;
        
        metrics.path_length = calculatePathLength(plan.trajectory_);
        metrics.joint_movement = calculateJointMovement(plan.trajectory_);
        metrics.smoothness = calculateSmoothness(plan.trajectory_);
        metrics.waypoint_count = plan.trajectory_.joint_trajectory.points.size();
        
        // 存储评估结果
        path_metrics_.push_back(metrics);
        
        // 输出当前路径的质量指标
        RCLCPP_INFO(this->get_logger(), "路径质量评估结果:");
        RCLCPP_INFO(this->get_logger(), "- 路径长度: %.3f 米", metrics.path_length);
        RCLCPP_INFO(this->get_logger(), "- 关节总移动量: %.3f 弧度", metrics.joint_movement);
        RCLCPP_INFO(this->get_logger(), "- 路径平滑度: %.3f", metrics.smoothness);
        RCLCPP_INFO(this->get_logger(), "- 路径点数量: %d", metrics.waypoint_count);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKPlanningTest>("ik_planning_test");
    
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