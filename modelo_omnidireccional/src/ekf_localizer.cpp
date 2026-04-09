#include "ekf_localizer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

using namespace robmovil;

EKFLocalizer::EKFLocalizer()
: Node("ekf_localizer"),
  mu_(Eigen::Vector3d::Zero()),
  odom_initialized_(false),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    // Covarianza inicial: poca incertidumbre en la pose de partida
    sigma_ = Eigen::Matrix3d::Identity() * 0.01;

    // Ruido de proceso Q: incertidumbre del modelo cinematico
    // (la odometria acumula error, mayor incertidumbre en posicion que en orientacion)
    Q_ = Eigen::DiagonalMatrix<double,3>(
        std::pow(0.05, 2),          // sigma_x^2
        std::pow(0.05, 2),          // sigma_y^2
        std::pow(2.0 * M_PI/180, 2) // sigma_theta^2 (2 grados)
    );

    // Ruido de medicion R: el laser es mas confiable que la odometria
    R_ = Eigen::DiagonalMatrix<double,2>(
        std::pow(0.1, 2),               // sigma_range^2  (10 cm)
        std::pow(5.0 * M_PI/180, 2)     // sigma_bearing^2 (5 grados)
    );

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot/odometry", rclcpp::QoS(10),
        std::bind(&EKFLocalizer::on_odometry, this, std::placeholders::_1));

    // QoS transient_local para recibir /posts aunque el nodo arranque despues de la sim
    auto qos_latched = rclcpp::QoS(1).transient_local();
    posts_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/posts", qos_latched,
        std::bind(&EKFLocalizer::on_posts, this, std::placeholders::_1));

    landmarks_sub_ = this->create_subscription<robmovil_msgs::msg::LandmarkArray>(
        "/landmarks", rclcpp::QoS(10),
        std::bind(&EKFLocalizer::on_landmarks, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "EKF node ready");
}

void EKFLocalizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!posts_map_.empty())
        return; // ya cargado

    for (const auto &p : msg->poses)
        posts_map_.emplace_back(p.position.x, p.position.y);

    RCLCPP_INFO(this->get_logger(), "Map loaded: %zu posts", posts_map_.size());
}

void EKFLocalizer::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    /*
    PREDICCIÓN: integra las velocidades del robot (frame robot) recibidas por
    /robot/odometry para propagar el estado mu_ y la covarianza sigma_ al
    frame map. Luego publica la TF map→base_link_ekf.
    */
    rclcpp::Time now(msg->header.stamp);

    if (!odom_initialized_) {
        // Inicializar mu_ desde map→odom para que el EKF arranque en la
        // posición real del robot en el frame map, no en el origen.
        try {
            auto tf = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
            auto &t = tf.transform.translation;
            auto &r = tf.transform.rotation;
            double siny = 2.0 * (r.w * r.z + r.x * r.y);
            double cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z);
            mu_ << t.x, t.y, std::atan2(siny, cosy);
            RCLCPP_INFO(this->get_logger(),
                "EKF initialized from map→odom: x=%.3f y=%.3f theta=%.3f",
                mu_(0), mu_(1), mu_(2));
        } catch (const tf2::TransformException &) {
            RCLCPP_WARN(this->get_logger(),
                "map→odom not available yet, initializing at origin");
        }
        last_odom_time_  = now;
        odom_initialized_ = true;
        return;
    }

    double delta_t = (now - last_odom_time_).seconds();
    last_odom_time_ = now;

    if (delta_t <= 0.0)
        return;

    _predict(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.angular.z,
        delta_t
    );

    publish_tf(now);
}

void EKFLocalizer::_predict(double vx, double vy, double omega, double dt)
{
    /*
    Aplica el modelo de movimiento f(x,u) y propaga la covarianza.
    Las velocidades vx, vy, omega están en frame robot; se rotan a frame map
    antes de integrar con Euler. La covarianza se propaga con el Jacobiano F y
    se incrementa con el ruido de proceso Q_.
    */
    // Modelo de movimiento f(x, u): integra velocidades [base_link] al frame [map]
    double theta = mu_(2);
    mu_(0) += (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    mu_(1) += (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
    mu_(2)  = normalize_angle(mu_(2) + omega * dt);

    // Propagacion de covarianza: sigma = F * sigma * F^T + Q
    Eigen::Matrix3d F = _motion_jacobian(theta, vx, vy, dt);
    sigma_ = F * sigma_ * F.transpose() + Q_;
}

Eigen::Matrix3d EKFLocalizer::_motion_jacobian(
    double theta, double vx, double vy, double dt) const
{
    /*
    Linealización de f alrededor de mu_{k-1}: F = df/dx evaluado en (theta, vx, vy, dt).
    Solo la tercera columna es no trivial (derivada respecto a theta).
    */
    // Jacobiano F = df/dx del modelo de movimiento evaluado en (theta, vx, vy, dt)
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = (-vx * std::sin(theta) - vy * std::cos(theta)) * dt;
    F(1, 2) = ( vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    return F;
}

void EKFLocalizer::on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
    /*
    CORRECCIÓN: recibe los centroides detectados por el LiDAR en /landmarks
    (frame base_link) y las posiciones reales de los postes del mapa en /posts
    (frame map). Para cada landmark detectado: asocia al poste más compatible
    con el estado predicho y aplica un paso de actualización EKF.
    */
    if (posts_map_.empty())
        return;

    for (const auto &lm : msg->landmarks) {
        int post_idx = associate(lm.range, lm.bearing);
        if (post_idx < 0)
            continue;
        _correct(lm.range, lm.bearing, post_idx);
    }
}

void EKFLocalizer::_correct(double z_range, double z_bearing, int post_idx)
{
    /*
    Aplica un paso completo de corrección EKF para el poste post_idx:
    calcula la innovación (diferencia entre medición real y esperada),
    la ganancia de Kalman K, y actualiza mu_ y sigma_.
    */
    // Medicion esperada h(mu^-) y jacobiano H para el poste asociado
    auto [h_mu, H] = _observation_model(post_idx);

    // Innovacion: diferencia entre medicion real y esperada
    Eigen::Vector2d innovation;
    innovation(0) = z_range   - h_mu(0);
    innovation(1) = normalize_angle(z_bearing - h_mu(1));

    // Ganancia de Kalman y actualizacion de estado y covarianza
    Eigen::Matrix<double,3,2> K = _kalman_gain(H);
    mu_    = mu_ + K * innovation;
    mu_(2) = normalize_angle(mu_(2));
    sigma_ = (Eigen::Matrix3d::Identity() - K * H) * sigma_;
}

std::pair<Eigen::Vector2d, Eigen::Matrix<double,2,3>>
EKFLocalizer::_observation_model(int post_idx) const
{
    /*
    Calcula la medición esperada h(mu^-) = [rho, beta] y el Jacobiano
    H = dh/dx para el poste post_idx desde el estado predicho mu_.
    rho: distancia euclidea al poste. beta: bearing relativo al heading del robot.
    */
    // h(mu^-): rango y bearing esperados al poste desde el estado predicho
    double delta_x = posts_map_[post_idx].x - mu_(0);
    double delta_y = posts_map_[post_idx].y - mu_(1);
    double rho     = std::sqrt(delta_x*delta_x + delta_y*delta_y);

    Eigen::Vector2d h_mu(rho, normalize_angle(std::atan2(delta_y, delta_x) - mu_(2)));

    // Jacobiano H = dh/dx evaluado en mu^-
    Eigen::Matrix<double,2,3> H;
    H << -delta_x/rho,       -delta_y/rho,       0,
          delta_y/(rho*rho), -delta_x/(rho*rho), -1;

    return {h_mu, H};
}

Eigen::Matrix<double,3,2>
EKFLocalizer::_kalman_gain(const Eigen::Matrix<double,2,3>& H) const
{
    /*
    Calcula la ganancia de Kalman K = sigma_ * H^T * S^{-1},
    donde S = H * sigma_ * H^T + R_ es la covarianza de la innovación.
    K grande → se confía más en el sensor; K chico → se confía más en la predicción.
    */
    // K = sigma * H^T * (H * sigma * H^T + R)^{-1}
    Eigen::Matrix2d S = H * sigma_ * H.transpose() + R_;
    return sigma_ * H.transpose() * S.inverse();
}

int EKFLocalizer::associate(double z_range, double z_bearing) const
{
    // Devuelve el índice del poste asociado al centroide estimado 

    double robot_x = mu_(0), robot_y = mu_(1), theta = mu_(2);
    double best_dist = 2.0; // umbral de asociacion
    int best_idx = -1;

    for (int i = 0; i < static_cast<int>(posts_map_.size()); ++i) {
        double post_x = posts_map_[i].x;
        double post_y = posts_map_[i].y;
        double delta_x = post_x - robot_x;
        double delta_y = post_y - robot_y;

        double expected_range   = std::sqrt(delta_x*delta_x + delta_y*delta_y);
        double expected_bearing = normalize_angle(std::atan2(delta_y, delta_x) - theta);

        double match_score = std::abs(z_range - expected_range) + std::abs(z_bearing - expected_bearing);
        if (match_score < best_dist) {
            best_dist = match_score;
            best_idx  = i;
        }
    }

    return best_idx;
}

void EKFLocalizer::publish_tf(const rclcpp::Time &stamp)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = stamp;
    t.header.frame_id = "map";
    t.child_frame_id  = "base_link_ekf";

    t.transform.translation.x = mu_(0);
    t.transform.translation.y = mu_(1);
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, mu_(2));
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

double EKFLocalizer::normalize_angle(double a)
{
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}
