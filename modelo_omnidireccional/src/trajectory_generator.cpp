#include "trajectory_generator.h"

using namespace robmovil;

SquareTrajectoryGenerator::SquareTrajectoryGenerator(double side, int points_per_side)
: side_(side), points_per_side_(points_per_side)
{}

std::vector<Waypoint> SquareTrajectoryGenerator::generate() const
{
    // Esquinas recorridas en sentido antihorario segun la figura del PDF:
    //   (2,-2) → (2,2) → (-2,2) → (-2,-2) → (vuelta a inicio)
    double half = side_ / 2.0;
    double corners[][2] = {
        { half, -half},
        { half,  half},
        {-half,  half},
        {-half, -half},
    };

    std::vector<Waypoint> waypoints;

    for (int edge = 0; edge < 4; edge++) {
        double x0 = corners[edge][0];
        double y0 = corners[edge][1];
        double x1 = corners[(edge + 1) % 4][0];
        double y1 = corners[(edge + 1) % 4][1];

        for (int i = 0; i < points_per_side_; i++) {
            double t = static_cast<double>(i) / points_per_side_;
            double x = x0 + t * (x1 - x0);
            double y = y0 + t * (y1 - y0);

            // Orientacion "opuesta al centro": atan2(y,x) da la direccion
            // radialmente alejada del origen (0,0).
            double theta = std::atan2(y, x);

            waypoints.push_back({x, y, theta});
        }
    }

    return waypoints;
}