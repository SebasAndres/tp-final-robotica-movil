#pragma once

namespace robmovil {

// Punto 2D en metros (usado en detección laser y mapa de postes)
struct Point {
    double x, y;
    Point() = default;
    Point(double x, double y) : x(x), y(y) {}
};

// Pose 2D con orientación (usado en trayectorias)
struct Waypoint {
    double x, y, theta;
};

} // namespace robmovil
