#include "omni_odometry.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using namespace robmovil;

OmniOdometry::OmniOdometry()
: Node("omni_odometry"),
  x_(0.0), y_(0.0), theta_(0.0),
  ticks_initialized_(false)
{
    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/robot/cmd_vel", 
        rclcpp::QoS(10),
        std::bind(&OmniOdometry::on_cmd_vel, this, std::placeholders::_1)
    );
    encoders_sub_ = this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>(
        "/robot/encoders", 
        rclcpp::QoS(10),
        std::bind(&OmniOdometry::on_encoders, this, std::placeholders::_1)
    );

    // Publishers - velocidad de cada rueda
    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>(
        "/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>(
        "/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
    pub_rl_ = this->create_publisher<std_msgs::msg::Float64>(
        "/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
    pub_rr_ = this->create_publisher<std_msgs::msg::Float64>(
        "/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));

    // Publishers - odometria
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/robot/odometry", rclcpp::QoS(10));

    // TF broadcaster (odom -> base_link)
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    std::fill_n(last_ticks_, 4, 0);
}

void OmniOdometry::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    /*
    Se ejecuta al recibir un mensaje Twist por el tópico /robot/cmd_vel.
    Transforma el twist a velocidades para cada una de las ruedas que envía 
    en /robot/.../cmd_vel
    */

    double linear_velocity_x = msg->linear.x;
    double linear_velocity_y = msg->linear.y;
    double angular_velocity_z = msg->angular.z;

    // [<< Ec. 20 paper>>] Cinematica inversa 
    double front_right_wheel_angular_velocity = (1/WHEEL_RADIUS) * (linear_velocity_x + linear_velocity_y + (L_X + L_Y) * angular_velocity_z);
    double rear_left_wheel_angular_velocity = (1/WHEEL_RADIUS) * (linear_velocity_x + linear_velocity_y - (L_X + L_Y) * angular_velocity_z);
    double rear_right_wheel_angular_velocity = (1/WHEEL_RADIUS) * (linear_velocity_x - linear_velocity_y + (L_X + L_Y) * angular_velocity_z);
    double front_left_wheel_angular_velocity = (1/WHEEL_RADIUS) * (linear_velocity_x - linear_velocity_y - (L_X + L_Y) * angular_velocity_z);

    _publish_wheel_velocities(
        front_right_wheel_angular_velocity,
        rear_left_wheel_angular_velocity,
        rear_right_wheel_angular_velocity,
        front_left_wheel_angular_velocity
    );
}

void OmniOdometry::_publish_wheel_velocities(
    double front_right_wheel_angular_velocity,
    double rear_left_wheel_angular_velocity,
    double rear_right_wheel_angular_velocity,
    double front_left_wheel_angular_velocity    
){
    std_msgs::msg::Float64 msg_fl;
    msg_fl.data = front_left_wheel_angular_velocity;
    pub_fl_->publish(msg_fl);

    std_msgs::msg::Float64 msg_fr;
    msg_fr.data = front_right_wheel_angular_velocity;
    pub_fr_->publish(msg_fr);

    std_msgs::msg::Float64 msg_rl;
    msg_rl.data = rear_left_wheel_angular_velocity;
    pub_rl_->publish(msg_rl);

    std_msgs::msg::Float64 msg_rr;
    msg_rr.data = rear_right_wheel_angular_velocity;
    pub_rr_->publish(msg_rr);
}

void OmniOdometry::on_encoders(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg)
{
    /*
    Se ejecuta al recibir un mensaje de /robot/encoders/MultiEncoderTicks
    Calcula las velocidades + pose desde Odom y la publica en /robot/odometry
    Además publica la transformacion odom->base_link.
    */

    if (!ticks_initialized_) {
        ticks_initialized_ = true;
        for (int i = 0; i < 4; i++)
            last_ticks_[i] = msg->ticks[i];
        last_ticks_time_ = msg->header.stamp;
        return;
    }

    rclcpp::Time current_time(msg->header.stamp);
    double delta_t = (current_time - last_ticks_time_).seconds();

    if (delta_t <= 0.0)
        return;

    std::array<double, 4> wheel_distances;
    _get_wheel_distances_from_encoder_msg(msg, wheel_distances);

    _update_pose_and_vel(wheel_distances);
    _publish_odometry(delta_t, current_time);
    _publish_odom_base_link_tf(current_time);    

    _update_internal_ticks_state(current_time, msg);
}

void OmniOdometry::_update_internal_ticks_state(
    rclcpp::Time current_time,
    const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg
){
    for (size_t i = 0; i < 4; i++)
        last_ticks_[i] = msg->ticks[i];
    last_ticks_time_ = current_time;
}

void OmniOdometry::_get_wheel_distances_from_encoder_msg(
    const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr msg,
    std::array<double, 4>& wheel_distances
){
    // Distancia lineal incremental recorrida por cada una de las cuatro ruedas
    int32_t delta_encoder_ticks;
    for (size_t i = 0; i < 4; i++){
        delta_encoder_ticks = msg->ticks[i] - last_ticks_[i];
        wheel_distances[i] = delta_encoder_ticks * 2.0 * M_PI * WHEEL_RADIUS / TICKS_PER_REV;
    }   
}

void OmniOdometry::_update_pose_and_vel(
    const std::array<double, 4>& wheel_distances
){
    // [<<Ec. 22 a 24 >>] Cinematica directa
    longitudinal_velocity_ = ( wheel_distances[0]+wheel_distances[1]+wheel_distances[2]+wheel_distances[3]) / 4.0;
    transversal_velocity_ = (-wheel_distances[0]+wheel_distances[1]+wheel_distances[2]-wheel_distances[3]) / 4.0;
    angular_velocity_   = (-wheel_distances[0]+wheel_distances[1]-wheel_distances[2]+wheel_distances[3]) / (4.0*(L_X+L_Y));

    x_ += longitudinal_velocity_ * cos(theta_) - transversal_velocity_ * sin(theta_);
    y_ += longitudinal_velocity_ * sin(theta_) + transversal_velocity_ * cos(theta_);
    theta_ += angular_velocity_;

    robot_orientation_quaternion_.setRPY(0, 0, theta_);
}

void OmniOdometry::_publish_odom_base_link_tf(
    const rclcpp::Time& stamp
){
    // Publica transformacion odom->base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(robot_orientation_quaternion_);

    tf_broadcaster_->sendTransform(t);
}

void OmniOdometry::_publish_odometry(
    double delta_t,
    const rclcpp::Time& stamp
){
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.twist.twist.linear.x = longitudinal_velocity_ / delta_t;
    odom_msg.twist.twist.linear.y = transversal_velocity_ / delta_t;
    odom_msg.twist.twist.angular.z = angular_velocity_ / delta_t;

    odom_msg.pose.pose.orientation = tf2::toMsg(robot_orientation_quaternion_);

    pub_odometry_->publish(odom_msg);
}