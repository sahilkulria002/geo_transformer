#include <GeographicLib/LocalCartesian.hpp>
#include <iostream>

int main() {
    // WGS84 coordinates for some point
    double lat = 40.6892, lon = -74.0445, alt = 0.0;
    // Set origin (Statue of Liberty)
    GeographicLib::LocalCartesian proj(lat, lon, alt);
    double x, y, z;
    // Convert the origin to local coordinates (should be 0,0,0)
    proj.Forward(lat, lon, alt, x, y, z);
    std::cout << "Local coordinates: x=" << x << ", y=" << y << ", z=" << z << std::endl;
    return 0;
}
