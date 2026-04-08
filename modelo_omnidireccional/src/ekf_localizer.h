#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <vector>

#include "types.h"

namespace robmovil {

class EKFLocalizer : public rclcpp::Node
{
/*
Declaración del nodo.

Este nodo implementa un Filtro de Kalman Extendido (EKF) para localización
del robot en el frame map, fusionando odometría con observaciones de landmarks.

Estado interno: mu = [x, y, theta] en frame map.

- Predicción (on_odometry): integra las velocidades lineales y angular
  publicadas en /robot/odometry para propagar el estado y la covarianza
  mediante el Jacobiano F del modelo de movimiento.

- Corrección (on_landmarks): para cada landmark detectado en /landmarks,
  realiza asociación de datos por vecino más cercano contra el mapa de postes
  (/posts). Calcula la innovación en espacio [range, bearing] y aplica la
  ganancia de Kalman para corregir mu y sigma.

- Mapa de postes (on_posts): recibe una única vez la lista de postes conocidos
  en frame map desde el tópico /posts (geometry_msgs/PoseArray).

Publica:
  - La transformación map → base_link_ekf al tópico /tf, representando la
    pose estimada por el EKF del centro del robot en el frame map.
*/

public:
    EKFLocalizer();

private:
    void on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    
    void on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg);
    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Predicción: propaga mu_ y sigma_ con el modelo de movimiento
    void _predict(double vx, double vy, double omega, double dt);

    // Jacobiano del modelo de movimiento F = df/dx evaluado en (theta, vx, vy, dt)
    Eigen::Matrix3d _motion_jacobian(double theta, double vx, double vy, double dt) const;

    // Corrección: aplica un paso de actualización EKF para el poste post_idx
    void _correct(double z_range, double z_bearing, int post_idx);

    // Modelo de observación: devuelve h(mu_) y H para el poste post_idx
    std::pair<Eigen::Vector2d, Eigen::Matrix<double,2,3>> _observation_model(int post_idx) const;

    // Ganancia de Kalman K = sigma_ * H^T * (H * sigma_ * H^T + R_)^{-1}
    Eigen::Matrix<double,3,2> _kalman_gain(const Eigen::Matrix<double,2,3>& H) const;

    // Asociación de datos: devuelve índice del poste más compatible o -1
    int associate(double z_range, double z_bearing) const;

    // Publica TF map → base_link_ekf
    void publish_tf(const rclcpp::Time &stamp);

    static double normalize_angle(double a);

    // Pose estimada del robot en frame map: [x (m), y (m), theta (rad)]
    Eigen::Vector3d mu_;

    // Covarianza de la pose estimada (3x3). Crece con la predicción (Q_)
    // y se reduce con cada corrección por landmark.
    Eigen::Matrix3d sigma_;

    // Ruido de proceso (3x3 diagonal): modela el error acumulado del modelo
    // cinemático por paso de integración — cuánto se desconfía de la odometría.
    Eigen::Matrix3d Q_;

    // Ruido de medición (2x2 diagonal): modela el error del sensor laser al
    // estimar [range (m), bearing (rad)] de cada landmark.
    Eigen::Matrix2d R_;

    // Posiciones conocidas de los postes en frame map: cargadas una vez desde /posts.
    std::vector<Point> posts_map_;

    // Timestamp de la última odometría procesada, para calcular delta_t.
    rclcpp::Time last_odom_time_;

    // Evita integrar un delta_t inválido en el primer mensaje de odometría.
    bool odom_initialized_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr posts_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmarks_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

} // namespace robmovil
