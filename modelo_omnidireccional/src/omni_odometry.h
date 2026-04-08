#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robmovil_msgs/msg/multi_encoder_ticks.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace robmovil {

class OmniOdometry : public rclcpp::Node
{
/*
Declaración del nodo.

Este nodo se encarga de:
- Traducir los comandos de velocidad Twist (/robot/cmd_vel) a velocidades 
  para cada una de sus ruedas usando el modelo cinemático del Paper y  
  publicarlas en (/robot/.../cmd_vel). 

- Computar y publicar la odometría del robot en base a los sensores encoders 
  (/robot/encoders) de las rueda al moverse. 
  Publica:
    1. Al tópico /robot/odometry la pose y las velocidades respecto al origen de odom (para EKF).
    2. Al tópico /tf la pose respecto a odom como la transformación (odom->base_link, 
       i.e. la posición del centro del robot en coordenadas del frame odom donde arrancó).   
*/

public:
    OmniOdometry();

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void on_encoders(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg);

private:
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::MultiEncoderTicks>::SharedPtr encoders_sub_;

    // Publishers - velocidad de cada rueda
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rl_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rr_;

    // Publishers - odometria
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Pose estimada
    double x_, y_, theta_;
    tf2::Quaternion robot_orientation_quaternion_;

    // Velocities
    double longitudinal_velocity_, transversal_velocity_, angular_velocity_;

    // Estado de encoders
    bool ticks_initialized_;
    int32_t last_ticks_[4];
    rclcpp::Time last_ticks_time_;

    // Parametros del robot Mecanum
    static constexpr double WHEEL_RADIUS = 0.05;       // 50mm
    static constexpr double L_X = 0.175;                // mitad del largo (350mm/2)
    static constexpr double L_Y = 0.175;                // mitad del ancho (350mm/2)
    static constexpr double TICKS_PER_REV = 500.0;

    void _get_wheel_distances_from_encoder_msg(
        const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg,
        std::array<double, 4>& wheel_distances
    );
    void _update_pose_and_vel(const std::array<double, 4>& wheel_distances);
    void _publish_odometry(double delta_t, const rclcpp::Time& stamp);
    void _publish_odom_base_link_tf(const rclcpp::Time& stamp);
    void _update_internal_ticks_state(
      rclcpp::Time current_time, 
      const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg
    );
    void _publish_wheel_velocities(
      double front_right_wheel_angular_velocity, 
      double rear_left_wheel_angular_velocity, 
      double rear_right_wheel_angular_velocity, 
      double front_left_wheel_angular_velocity
    );
  };

} // namespace robmovil
