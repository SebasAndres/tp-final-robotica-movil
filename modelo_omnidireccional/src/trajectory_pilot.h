#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "trajectory_generator.h"

// Set to 1 to use EKF pose (map->base_link_ekf), 0 for odometry (map->base_link)
#define USE_EKF 1

namespace robmovil {

class TrajectoryPilot : public rclcpp::Node
{
/*
Nodo de seguimiento de trayectoria.

Recibe una trayectoria generada externamente (TrajectoryGenerator) y la
recorre enviando comandos de velocidad a /robot/cmd_vel mediante un
controlador proporcional con seleccion Pursuit-Based de waypoints.
*/

public:
    explicit TrajectoryPilot(std::unique_ptr<TrajectoryGenerator> generator);

private:
    void control_loop();

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Waypoints
    std::vector<Waypoint> waypoints_;
    size_t current_wp_;

    // Gains
    double kp_x_;
    double kp_y_;
    double kp_theta_;

    // Tolerance to advance to next waypoint
    double position_tolerance_;
    double angle_tolerance_;

    void _publish_waypoints_into_debug_topic();

    double normalize_angle(double angle) const;

    void _check_persuit_based_waypoint_checkpoint(
        double error_x_map, 
        double error_y_map, 
        double error_theta
    );

    geometry_msgs::msg::Twist _get_velocity_twist_cmd(
        double error_x_robot,
        double error_y_robot,
        double error_theta
    );
};

} // namespace robmovil