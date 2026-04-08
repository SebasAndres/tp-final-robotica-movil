#include "trajectory_pilot.h"
#include "trajectory_generator.h"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto generator = std::make_unique<robmovil::SquareTrajectoryGenerator>(2.0, 20);
    rclcpp::spin(std::make_shared<robmovil::TrajectoryPilot>(std::move(generator)));
    rclcpp::shutdown();
    return 0;
}
