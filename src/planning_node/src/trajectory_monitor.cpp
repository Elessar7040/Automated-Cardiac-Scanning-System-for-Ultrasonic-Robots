/**
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-02 15:55:20
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-20 14:10:00
 * @FilePath: /planning_control_node/src/planning_node/src/trajectory_monitor.cpp
 * @Description: 轨迹监控节点，用于记录和监控机器人关节运动数据
 * 主要功能：
 * - 订阅并记录六自由度机械臂控制器状态数据 (/russ_group_controller/state)
 * - 订阅并记录关节状态数据 (/joint_states)
 * - 订阅规划轨迹数据 (/display_planned_path)
 * - 将实际位置和目标位置数据记录到CSV文件
 * - 支持从程序启动开始的连续时间记录
 * - 提供轨迹插值功能以获取准确的目标位置
 *
 * 输出文件：
 * - arm_trajectory.csv: 记录控制器状态数据
 * - joint_states.csv: 记录关节状态数据
 */
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>
#include <vector>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class TrajectoryMonitor : public rclcpp::Node
{
public:
    TrajectoryMonitor() : Node("trajectory_monitor")
    {
        // 订阅执行的轨迹
        trajectory_sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
            "/display_planned_path", 1,
            std::bind(&TrajectoryMonitor::trajectory_callback, this, std::placeholders::_1));

        // 订阅控制器状态
        arm_controller_state_sub_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/russ_group_controller/state", 10,
            std::bind(&TrajectoryMonitor::arm_controller_state_callback, this, std::placeholders::_1));

        // 订阅关节状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryMonitor::joint_state_callback, this, std::placeholders::_1));

        // 初始化标志和容器
        trajectory_executing_ = false;
        planned_trajectory_.clear();
        planned_trajectory_joint_names_.clear();
        start_time_ = this->now();

        // 创建CSV日志文件 - 控制器状态
        arm_log_file_.open("arm_trajectory.csv");
        arm_log_file_ << "timestamp,joint_name,actual_position,actual_velocity,actual_acceleration,desired_position,desired_velocity,desired_acceleration,error_position,error_velocity,error_acceleration\n";

        // 创建CSV日志文件 - 关节状态
        joint_states_file_.open("joint_states.csv");
        joint_states_file_ << "timestamp,joint_name,position,velocity,effort\n";

        RCLCPP_INFO(this->get_logger(), "六自由度机械臂轨迹监控节点已启动");
    }

    ~TrajectoryMonitor()
    {
        if (arm_log_file_.is_open()) {
            arm_log_file_.close();
        }
        if (joint_states_file_.is_open()) {
            joint_states_file_.close();
        }
    }

private:
    rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_controller_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    std::vector<std::string> planned_trajectory_joint_names_;  // 存储规划轨迹的关节名称
    std::ofstream arm_log_file_;
    std::ofstream joint_states_file_;
    
    // 存储规划轨迹，用于比较实际执行情况
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> planned_trajectory_;
    rclcpp::Time trajectory_start_time_;
    rclcpp::Time start_time_;
    double trajectory_offset_time_;  // 添加轨迹偏移时间
    bool trajectory_executing_ = false;

    void trajectory_callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
    {
        if (msg->trajectory.empty() || msg->trajectory[0].joint_trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空轨迹");
            return;
        }

        const auto& trajectory = msg->trajectory[0].joint_trajectory;
        planned_trajectory_joint_names_ = trajectory.joint_names;
        planned_trajectory_ = trajectory.points;
        trajectory_start_time_ = this->now();
        trajectory_executing_ = true;
        
        // 记录轨迹开始时相对于程序启动的时间偏移
        trajectory_offset_time_ = (trajectory_start_time_ - start_time_).seconds();

        RCLCPP_INFO(this->get_logger(), "收到新的轨迹，包含 %zu 个点，时间偏移: %.3f", 
            planned_trajectory_.size(), trajectory_offset_time_);
    }

    void arm_controller_state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        // 计算相对于程序启动的时间
        double time_from_start = (this->now() - start_time_).seconds();
        
        // 检查消息是否为空
        if (!msg || msg->joint_names.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "收到空的控制器状态消息");
            return;
        }

        // 每100条消息输出一次状态信息
        static int message_count = 0;
        if (message_count++ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "已收到 %d 条控制器状态消息", message_count);
        }

        // 只有当轨迹正在执行时才查找对应的目标点
        trajectory_msgs::msg::JointTrajectoryPoint target_point;
        bool found_target = false;
        double time_since_trajectory = 0.0;

        if (trajectory_executing_ && !planned_trajectory_.empty()) {
            time_since_trajectory = (this->now() - trajectory_start_time_).seconds();
            
            // 检查轨迹是否执行完成
            if (time_since_trajectory > 
                (planned_trajectory_.back().time_from_start.sec +
                 planned_trajectory_.back().time_from_start.nanosec * 1e-9)) {
                trajectory_executing_ = false;
                RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
            } else {
                // 在轨迹点中查找当前时间对应的目标点进行插值
                for (size_t k = 0; k < planned_trajectory_.size() - 1; ++k) {
                    double t1 = planned_trajectory_[k].time_from_start.sec + 
                              planned_trajectory_[k].time_from_start.nanosec * 1e-9;
                    double t2 = planned_trajectory_[k+1].time_from_start.sec + 
                              planned_trajectory_[k+1].time_from_start.nanosec * 1e-9;

                    if (time_since_trajectory >= t1 && time_since_trajectory <= t2) {
                        double alpha = (time_since_trajectory - t1) / (t2 - t1);
                        
                        target_point.positions.resize(planned_trajectory_[k].positions.size());
                        target_point.velocities.resize(planned_trajectory_[k].velocities.size());
                        
                        for (size_t j = 0; j < target_point.positions.size(); ++j) {
                            target_point.positions[j] = planned_trajectory_[k].positions[j] * (1-alpha) +
                                                      planned_trajectory_[k+1].positions[j] * alpha;
                            
                            if (j < target_point.velocities.size()) {
                                target_point.velocities[j] = planned_trajectory_[k].velocities[j] * (1-alpha) +
                                                           planned_trajectory_[k+1].velocities[j] * alpha;
                            }
                        }
                        found_target = true;
                        break;
                    }
                }
            }
        }

        try {
            // 记录每个关节的实际数据和目标数据
            for (size_t i = 0; i < msg->joint_names.size(); ++i) {
                const std::string& joint_name = msg->joint_names[i];
                
                // 查找当前关节在规划轨迹中的索引
                int target_index = -1;
                if (found_target) {
                    auto it = std::find(planned_trajectory_joint_names_.begin(),
                                      planned_trajectory_joint_names_.end(),
                                      joint_name);
                    if (it != planned_trajectory_joint_names_.end()) {
                        target_index = std::distance(planned_trajectory_joint_names_.begin(), it);
                    }
                }
                
                // 写入数据
                arm_log_file_ << time_from_start << ","
                             << joint_name << ",";
                             
                // 实际位置
                if (i < msg->actual.positions.size()) {
                    arm_log_file_ << msg->actual.positions[i] << ",";
                } else {
                    arm_log_file_ << "0.0,";
                }
                
                // 实际速度
                if (i < msg->actual.velocities.size()) {
                    arm_log_file_ << msg->actual.velocities[i] << ",";
                } else {
                    arm_log_file_ << "0.0,";
                }
                
                // 实际加速度
                if (i < msg->actual.accelerations.size()) {
                    arm_log_file_ << msg->actual.accelerations[i] << ",";
                } else {
                    arm_log_file_ << "0.0,";
                }
                
                // 目标位置
                if (found_target && target_index >= 0 && target_index < static_cast<int>(target_point.positions.size())) {
                    arm_log_file_ << target_point.positions[target_index] << ",";
                } else if (i < msg->desired.positions.size()) {
                    arm_log_file_ << msg->desired.positions[i] << ",";
                } else {
                    arm_log_file_ << "0.0,";
                }
                
                // 目标速度
                if (found_target && target_index >= 0 && target_index < static_cast<int>(target_point.velocities.size())) {
                    arm_log_file_ << target_point.velocities[target_index] << ",";
                } else if (i < msg->desired.velocities.size()) {
                    arm_log_file_ << msg->desired.velocities[i] << ",";
                } else {
                    arm_log_file_ << "0.0,";
                }

                // 目标加速度
                if (found_target && target_index >= 0 && target_index < static_cast<int>(target_point.accelerations.size()))
                {
                    arm_log_file_ << target_point.accelerations[target_index] << ",";
                }
                else if (i < msg->desired.accelerations.size())
                {
                    arm_log_file_ << msg->desired.accelerations[i] << ",";
                }
                else
                {
                    arm_log_file_ << "0.0,";
                }

                // 位置误差
                if (i < msg->error.positions.size()) {
                    arm_log_file_ << msg->error.positions[i];
                } else {
                    arm_log_file_ << "0.0";
                }
                
                arm_log_file_ << "\n";
            }
            
            // 确保数据立即写入文件
            arm_log_file_.flush();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "写入控制器数据时发生错误: %s", e.what());
        }
    }

    // 添加关节状态回调函数
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 计算相对于程序启动的时间
        double time_from_start = (this->now() - start_time_).seconds();
        
        // 检查消息是否为空
        if (!msg || msg->name.empty()) {
            return; // 不打印警告，因为这种情况很少发生
        }

        // 记录关节状态数据
        try {
            for (size_t i = 0; i < msg->name.size(); ++i) {
                joint_states_file_ << time_from_start << ","
                                 << msg->name[i] << ",";
                
                // 位置
                if (i < msg->position.size()) {
                    joint_states_file_ << msg->position[i] << ",";
                } else {
                    joint_states_file_ << "0.0,";
                }
                
                // 速度
                if (i < msg->velocity.size()) {
                    joint_states_file_ << msg->velocity[i] << ",";
                } else {
                    joint_states_file_ << "0.0,";
                }
                
                // 力矩
                if (i < msg->effort.size()) {
                    joint_states_file_ << msg->effort[i];
                } else {
                    joint_states_file_ << "0.0";
                }
                
                joint_states_file_ << "\n";
            }
            
            // 确保数据立即写入文件
            joint_states_file_.flush();
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "写入关节状态数据时发生错误: %s", e.what());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}