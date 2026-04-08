#pragma once

#include <vector>
#include <cmath>

#include "types.h"

namespace robmovil {

class TrajectoryGenerator
{
public:
    virtual ~TrajectoryGenerator() = default;
    virtual std::vector<Waypoint> generate() const = 0;
};

class SquareTrajectoryGenerator : public TrajectoryGenerator
{
public:
    SquareTrajectoryGenerator(double side, int points_per_side);
    std::vector<Waypoint> generate() const override;

private:
    double side_;
    int points_per_side_;
};

} // namespace robmovil