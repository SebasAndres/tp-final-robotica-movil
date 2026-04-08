#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "types.h"

namespace robmovil {

class LaserDetector : public rclcpp::Node
{
/*
Declaración del nodo:

Este nodo se encarga de:
- Recibir mensajes del tópico /robot/front_laser/scan y calcular mediante 
  clusterización los centroides de cada uno de los postes.
- Luego publica cada centroide al tópico /robmovil_msgs/msg/LandmarkArray.
*/

public:
    LaserDetector();

private:
    void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmarks_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Distancia maxima entre puntos del mismo cluster (metros)
    static constexpr double CLUSTER_DIST   = 0.1;
    // Rango maximo de deteccion (metros)
    static constexpr double MAX_RANGE      = 10.0;
    // Diametro esperado de un poste (metros) — se usa para filtrar clusters
    static constexpr double POST_DIAMETER  = 0.1;
    static constexpr double POST_TOL       = 0.08;

    void _load_cartesian_laser_points(
        std::vector<Point>& laser_points,
        const sensor_msgs::msg::LaserScan &scan
    ) const;
    std::vector<Point> _compute_clusters(const sensor_msgs::msg::LaserScan &scan) const;
    void _compute_centroids(std::vector<Point>& centroids, std::vector<std::vector<Point>>& cluster_points) const;
    void _join_cluster_points(std::vector<Point>& laser_points, std::vector<std::vector<Point>>& cluster_points) const;
    void _transform_landmarks_to_base_link_frame(
        const sensor_msgs::msg::LaserScan::SharedPtr msg,
        std::vector<Point>& clusters, 
        robmovil_msgs::msg::LandmarkArray& corrected_landmarks
    );
};

} // namespace robmovil
