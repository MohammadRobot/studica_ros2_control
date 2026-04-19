#ifndef DIFF_DRIVE_COMPONENT_H
#define DIFF_DRIVE_COMPONENT_H

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "diff_drive_odometry.h"
#include "studica_ros2_control/srv/set_data.hpp"
#include "titan.h"
#include "VMXPi.h"

namespace studica_ros2_control {

class DiffDrive : public rclcpp::Node {
public:
    // Selects which Titan CAN API is used each control cycle (set via titan_control_mode param).
    enum class TitanControlMode {
        OPEN_LOOP,  // SetSpeed: duty-cycle [-1,1], PID type 0
        VELOCITY,   // SetTargetVelocity: closed-loop RPM, PID type 1
        POSITION    // SetTargetDistance: accumulated encoder counts, PID type 2 + position hold
    };

    static std::shared_ptr<rclcpp::Node> initialize(rclcpp::Node *control, std::shared_ptr<DiffOdometry> odom, std::shared_ptr<VMXPi> vmx);
    explicit DiffDrive(const rclcpp::NodeOptions & options);
    DiffDrive(
        std::shared_ptr<VMXPi> vmx,
        std::shared_ptr<studica_ros2_control::DiffOdometry> odom,
        const std::string &name,
        const uint8_t &canID,
        const uint16_t &motor_freq,
        const uint16_t &ticks_per_rotation,
        const float &wheel_radius,
        const float &wheelbase,
        const float &width,
        const uint8_t &front_left,
        const uint8_t &front_right,
        const uint8_t &rear_left,
        const uint8_t &rear_right,
        const bool &invert_front_left,
        const bool &invert_front_right,
        const bool &invert_rear_left,
        const bool &invert_rear_right,
        const std::string &titan_control_mode
    );
    ~DiffDrive();

private:
    std::shared_ptr<VMXPi> vmx_;
    std::shared_ptr<studica_ros2_control::DiffOdometry> odom_;
    std::shared_ptr<studica_driver::Titan> titan_;
    rclcpp::Service<studica_ros2_control::srv::SetData>::SharedPtr service_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    uint8_t can_id_;
    uint16_t motor_freq_;
    float ticks_per_rotation_;
    float dist_per_tick_;
    float wheel_radius_;
    float wheelbase_;
    float width_;
    uint8_t fl_;
    uint8_t fr_;
    uint8_t rl_;
    uint8_t rr_;
    TitanControlMode titan_control_mode_;
    std::string titan_control_mode_name_;
    bool have_last_cmd_time_;
    rclcpp::Time last_cmd_time_;
    // Accumulated position targets in encoder counts; only used in POSITION mode.
    // Seeded from current encoder reading on mode switch to avoid a jump.
    double fl_target_counts_;
    double fr_target_counts_;
    double rl_target_counts_;
    double rr_target_counts_;
    
    void cmd(std::string params, std::shared_ptr<studica_ros2_control::srv::SetData::Request> request, std::shared_ptr<studica_ros2_control::srv::SetData::Response> response);
    void cmd_callback(std::shared_ptr<studica_ros2_control::srv::SetData::Request> request, std::shared_ptr<studica_ros2_control::srv::SetData::Response> response);
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_odometry();
    // Parses mode string, calls apply_titan_pid_mode(), and seeds position targets from current encoder counts.
    void set_titan_control_mode(const std::string &mode_name);
    // Sends SetPIDType and SetPositionHold to the Titan device to match titan_control_mode_.
    void apply_titan_pid_mode();
    // Converts m/s to RPM, clamped to int16 range. Returns 0 if wheel_radius_ is not set.
    int16_t linear_mps_to_rpm(double linear_mps) const;
    // Dispatches to SetSpeed / SetTargetVelocity / SetTargetDistance based on current mode.
    void command_motor(uint8_t motor, double linear_mps, double dt_sec, double *target_counts);
};

}  // namespace studica_ros2_control

#endif  // TITAN_COMPONENT_H
