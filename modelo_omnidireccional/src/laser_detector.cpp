#include "laser_detector.h"
#include <cmath>

using namespace robmovil;

LaserDetector::LaserDetector()
: Node("laser_detector"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/robot/front_laser/scan", rclcpp::QoS(10),
        std::bind(&LaserDetector::on_scan, this, std::placeholders::_1));

    landmarks_pub_ = this->create_publisher<robmovil_msgs::msg::LandmarkArray>(
        "/landmarks", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "LaserDetector ready");
}

void LaserDetector::on_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg
){
    auto clusters = _compute_clusters(*msg);
    
    // Transformar cada centroide de cluster desde el frame del laser (front_laser)
    // al frame del robot (base_link).
    // T: front_laser -> base_link 
    // T(centroid)
    robmovil_msgs::msg::LandmarkArray corrected_landmarks;
    corrected_landmarks.header = msg->header;
    corrected_landmarks.header.frame_id = "base_link";
    _transform_landmarks_to_base_link_frame(msg, clusters, corrected_landmarks);

    landmarks_pub_->publish(corrected_landmarks);
}

void LaserDetector::_transform_landmarks_to_base_link_frame(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    std::vector<Point>& clusters, 
    robmovil_msgs::msg::LandmarkArray& corrected_landmarks
){
    for (const auto &c : clusters) {      
        geometry_msgs::msg::PointStamped pt_laser;
        pt_laser.header.stamp    = msg->header.stamp;
        pt_laser.header.frame_id = msg->header.frame_id; // "front_laser"
        pt_laser.point.x = c.x;
        pt_laser.point.y = c.y;
        pt_laser.point.z = 0.0;

        geometry_msgs::msg::PointStamped pt_robot;
        try {
            pt_robot = tf_buffer_.transform(
                pt_laser, "base_link", tf2::durationFromSec(0.1)
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "TF %s->base_link failed: %s",
                msg->header.frame_id.c_str(), ex.what());
            continue;
        }

        robmovil_msgs::msg::Landmark landmark;
        landmark.range   = static_cast<float>(std::sqrt(
            pt_robot.point.x * pt_robot.point.x +
            pt_robot.point.y * pt_robot.point.y));
        landmark.bearing = static_cast<float>(std::atan2(pt_robot.point.y, pt_robot.point.x));
        corrected_landmarks.landmarks.push_back(landmark);
    }
}

void LaserDetector::_load_cartesian_laser_points(
    std::vector<Point>& laser_points,
    const sensor_msgs::msg::LaserScan &scan
) const {
    // Devuelve un array de centroides de los clusteres en base al scan del LIDAR.
    // Los centroides están calculados con coordenadas del frame front_laser

    const int n = static_cast<int>(scan.ranges.size());
    for (int i = 0; i < n; ++i) {

        float linear_distance = scan.ranges[i];
        if (!std::isfinite(linear_distance) || linear_distance < scan.range_min || linear_distance > MAX_RANGE)
            continue;
        double angle = scan.angle_min + i * scan.angle_increment;

        // Coordenadas Polares -> Coordenadas Cartesianas
        double x = linear_distance * std::cos(angle);
        double y = linear_distance * std::sin(angle);
        laser_points.push_back({x,y});
    }
}

std::vector<Point> LaserDetector::_compute_clusters(const sensor_msgs::msg::LaserScan &scan) const
{
    // Paso 1: convertir rayos validos a puntos cartesianos en frame base_link
    std::vector<Point> laser_points; 
    _load_cartesian_laser_points(laser_points, scan);
    if (laser_points.empty())
        return {};

    // Paso 2: clusterizacion por distancia entre puntos consecutivos
    // { cluster 1, cluster 2 }
    // { {p1, p3 }, {..., p_i } }
    std::vector<std::vector<Point>> cluster_points;
    _join_cluster_points(
        laser_points,
        cluster_points
    );

    // Paso 3: filtrar clusters cuyo ancho angular corresponde a un poste de 0.1m
    // y calcular el centroide de cada uno
    std::vector<Point> centroids;
    _compute_centroids(centroids, cluster_points);

    return centroids;
}

void LaserDetector::_compute_centroids(
    std::vector<Point>& centroids,
    std::vector<std::vector<Point>>& cluster_points
) const {
    for (const auto &cl : cluster_points) {
        if (cl.size() < 2)
            continue;

        // Centroide
        double cx = 0, cy = 0;
        for (const auto &p : cl) { cx += p.x; cy += p.y; }
        cx /= cl.size();
        cy /= cl.size();

        // Ancho estimado del cluster: distancia entre el primer y ultimo punto
        const Point &first = cl.front();
        const Point &last  = cl.back();
        double width = std::sqrt(std::pow(last.x - first.x, 2) +
                                    std::pow(last.y - first.y, 2));

        // Conservar solo clusters con ancho compatible con un poste de 0.1m
        if (std::abs(width - POST_DIAMETER) < POST_TOL)
            centroids.push_back({cx, cy});
    }
}

void LaserDetector::_join_cluster_points(
    std::vector<Point>& laser_points,
    std::vector<std::vector<Point>>& cluster_points
) const {
    cluster_points.push_back({laser_points[0]});

    for (size_t i = 1; i < laser_points.size(); ++i) {
        const Point &prev = laser_points[i - 1];
        const Point &curr = laser_points[i];
        double dist = std::sqrt(std::pow(curr.x - prev.x, 2) +
                                std::pow(curr.y - prev.y, 2));
        
        if (dist < CLUSTER_DIST) {
            cluster_points.back().push_back(curr);
        } else {
            cluster_points.push_back({curr});
        }
    }
}