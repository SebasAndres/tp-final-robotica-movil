#include "trajectory_pilot.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace robmovil;

TrajectoryPilot::TrajectoryPilot(std::unique_ptr<TrajectoryGenerator> generator)
: Node("trajectory_pilot"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  current_wp_(0)
{
    // Ganancias del controlador proporcional (inciso 2.1)
    this->declare_parameter("kp_x", 1.0);
    this->declare_parameter("kp_y", 1.0);
    this->declare_parameter("kp_theta", 0.5);
    kp_x_ = this->get_parameter("kp_x").as_double();
    kp_y_ = this->get_parameter("kp_y").as_double();
    kp_theta_ = this->get_parameter("kp_theta").as_double();

    // Tolerancias para la seleccion Pursuit-Based (inciso 2.2)
    this->declare_parameter("position_tolerance", 0.1);
    this->declare_parameter("angle_tolerance", 0.15);
    this->declare_parameter("control_rate", 20.0);
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    double rate = this->get_parameter("control_rate").as_double();

    // Publisher - velocidad
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot/cmd_vel", rclcpp::QoS(10)
    );

    // Publisher - waypoints [debug]
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/trajectory_waypoints", rclcpp::QoS(10).transient_local()
    );
    
    waypoints_ = generator->generate();
    _publish_waypoints_into_debug_topic();

    // ?
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&TrajectoryPilot::control_loop, this)
    );
}

void TrajectoryPilot::_publish_waypoints_into_debug_topic(){
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = this->now();
    pose_array.header.frame_id = "map";
    for (const auto &wp : waypoints_) {
        geometry_msgs::msg::Pose p;
        p.position.x = wp.x;
        p.position.y = wp.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, wp.theta);
        p.orientation = tf2::toMsg(q);
        pose_array.poses.push_back(p);
    }
    waypoints_pub_->publish(pose_array);
    RCLCPP_INFO(
        this->get_logger(),
        "Trajectory loaded: %zu waypoints, Kp=(%.2f, %.2f, %.2f)",
        waypoints_.size(), kp_x_, kp_y_, kp_theta_
    );
}

void TrajectoryPilot::control_loop()
{
    if (current_wp_ >= waypoints_.size()) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Trajectory complete!");
        return;
    }

    const Waypoint &goal = waypoints_[current_wp_];

    #if USE_EKF
        static constexpr const char* kOdomFrame = "base_link_ekf";
    #else
        static constexpr const char* kOdomFrame = "base_link";
    #endif

    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_.lookupTransform("map", kOdomFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Waiting for map->base_link TF: %s", ex.what());
        return;
    }

    double curr_x     = tf.transform.translation.x;
    double curr_y     = tf.transform.translation.y;
    double curr_theta = tf2::getYaw(tf.transform.rotation);

    double error_x_map = goal.x - curr_x;
    double error_y_map = goal.y - curr_y;
    
    double error_x_robot =  std::cos(curr_theta) * error_x_map + std::sin(curr_theta) * error_y_map;
    double error_y_robot = -std::sin(curr_theta) * error_x_map + std::cos(curr_theta) * error_y_map;
    double error_theta = normalize_angle(goal.theta - curr_theta);

    geometry_msgs::msg::Twist cmd = _get_velocity_twist_cmd(error_x_robot, error_y_robot, error_theta);
    cmd_vel_pub_->publish(cmd);

    _check_persuit_based_waypoint_checkpoint(error_x_map, error_y_map, error_theta);
}

void TrajectoryPilot::_check_persuit_based_waypoint_checkpoint(
    double error_x_map, 
    double error_y_map, 
    double error_theta
){
    double pos_error = std::sqrt(error_x_map * error_x_map + error_y_map * error_y_map);
    if (pos_error < position_tolerance_ && std::abs(error_theta) < angle_tolerance_) {
        RCLCPP_INFO(
            this->get_logger(), 
            "Reached waypoint %zu/%zu",
            current_wp_ + 1, 
            waypoints_.size()
        );
        current_wp_++;
    }
}

geometry_msgs::msg::Twist TrajectoryPilot::_get_velocity_twist_cmd(
    double error_x_robot,
    double error_y_robot,
    double error_theta
){

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = kp_x_     * error_x_robot;
    cmd.linear.y  = kp_y_     * error_y_robot;
    cmd.angular.z = kp_theta_ * error_theta;

    return cmd;
}

double TrajectoryPilot::normalize_angle(double angle) const
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}