#include <rclcpp/rclcpp.hpp>
#include "ekf_localizer.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robmovil::EKFLocalizer>());
    rclcpp::shutdown();
    return 0;
}
