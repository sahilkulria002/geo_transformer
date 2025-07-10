# geo_transformer Package Documentation
#
# Version and Technology Stack

- **C++ Standard:** C++14 or newer (set by ROS 2 Humble and CMake)
- **ROS 2:** Humble Hawksbill (or later)
- **GeographicLib:** C++ library (system dependency)
- **Build System:** CMake (via ament_cmake)
- **Python:** 3.x (for test scripts)
- **Test Script Dependencies:** See `scripts/requirements.txt`

# Functional Details 
## Overview
`geo_transformer` is a ROS 2 C++ package that provides robust, service-based interfaces for converting between global GPS coordinates (WGS84: latitude, longitude, altitude) and a local Cartesian coordinate frame (x, y, z). It uses [GeographicLib](https://geographiclib.sourceforge.io/) for all geodetic calculations, ensuring high accuracy and reliability.


---

## Features
- **Set/Get Local Origin:**
  - Define the reference point (origin) for the local frame using latitude, longitude, and altitude.
  - Retrieve the current origin at any time.
- **Coordinate Conversion Services:**
  - Convert WGS84 (lat, lon, alt) to local (x, y, z).
  - Convert local (x, y, z) to WGS84 (lat, lon, alt).
- **Robust Error Handling:**
  - Validates all input ranges and provides clear error messages.
  - Handles exceptions from GeographicLib gracefully.
- **Python Test Script:**
  - End-to-end test script for all services, demonstrating correct usage and round-trip accuracy.
- **Launch File:**
  - Easily launch the node with ROS 2 launch system.
---


## validity proof - 
**this package** - 
'Origin set to (40.000000, -70.000000, 0.000000)'
Request(latitude=40.0, longitude=-74.0, altitude=0.0) --> Response(x=-341298.0292421531, y=7660.993295452623, z=-9130.016282695811
**https://tool-online.com/en/coordinate-converter.php**
origin - x = -4675243.087, y = 6656955.494
point -  x = -4964480.795, y = 7023419.246

## ###########################################################
# Technical Implementation Details

- **Node Implementation:**
  - The main node is implemented in C++ (`src/geo_transformation_node.cpp`) as a subclass of `rclcpp::Node`.
  - On startup, the node advertises four ROS 2 services: `/local_coordinate/set`, `/local_coordinate/get`, `/from_ll`, and `/to_ll`.
  - The node maintains internal state for the current origin (latitude, longitude, altitude) and a `GeographicLib::LocalCartesian` object for transformations.
  - The node uses smart pointers and exception handling for safety and robustness.

- **Service Handlers:**
  - Each service has a dedicated handler method:
    - `handle_set_origin`: Validates input, sets the origin, and initializes the transformation object.
    - `handle_get_origin`: Returns the current origin or NaN if not set.
    - `handle_from_ll`: Converts WGS84 to local, with input validation and error handling.
    - `handle_to_ll`: Converts local to WGS84, with error handling.
  - All handlers check that the origin is set before performing conversions.
  - All exceptions from GeographicLib are caught and reported in the service response.

- **Service Definitions:**
  - Custom `.srv` files define the request and response structure for each service.
  - All fields are standard ROS 2 types (float64, bool, string) for maximum compatibility.

- **Error Handling:**
  - Input values are checked for valid ranges (latitude, longitude, altitude).
  - If the origin is not set, conversion services return an error.
  - Unusual altitudes trigger a warning in the ROS log.
  - All exceptions are caught and reported in the response message.

- **Python Test Script:**
  - The provided script (`scripts/test_services.py`) demonstrates end-to-end usage of all services, including round-trip conversion checks.
  - The script can be used as a template for integrating the package into other Python-based ROS 2 nodes.

- **Launch File:**
  - The launch file (`launch/geo_transformer.launch.py`) allows easy startup of the node in a ROS 2 system.

- **Build and Install:**
  - The package uses standard ROS 2 build tools (`colcon`, `ament_cmake`).
  - All dependencies are declared in `CMakeLists.txt` and `package.xml`.
  - The node, service definitions, launch file, and test script are all installed for easy use.

---

## Services

### 1. `/local_coordinate/set` (`SetOrigin.srv`)
Set the local frame origin.
- **Request:**
  - `float64 latitude`   (must be in [-90, 90])
  - `float64 longitude`  (must be in [-180, 180])
  - `float64 altitude`   (meters, warning if outside [-500, 10000])
- **Response:**
  - `bool success`
  - `string message`

### 2. `/local_coordinate/get` (`GetOrigin.srv`)
Get the current local frame origin.
- **Request:**
  - *(empty)*
- **Response:**
  - `float64 latitude`
  - `float64 longitude`
  - `float64 altitude`
  - `bool success`
  - `string message`

### 3. `/from_ll` (`FromLL.srv`)
Convert WGS84 (lat, lon, alt) to local (x, y, z).
- **Request:**
  - `float64 latitude`   (must be in [-90, 90])
  - `float64 longitude`  (must be in [-180, 180])
  - `float64 altitude`   (meters, warning if outside [-500, 10000])
- **Response:**
  - `float64 x`
  - `float64 y`
  - `float64 z`
  - `bool success`
  - `string message`

### 4. `/to_ll` (`ToLL.srv`)
Convert local (x, y, z) to WGS84 (lat, lon, alt).
- **Request:**
  - `float64 x`
  - `float64 y`
  - `float64 z`
- **Response:**
  - `float64 latitude`
  - `float64 longitude`
  - `float64 altitude`
  - `bool success`
  - `string message`


---

## File Structure
- `src/geo_transformation_node.cpp` — Main C++ node implementation
- `srv/SetOrigin.srv`, `srv/GetOrigin.srv`, `srv/FromLL.srv`, `srv/ToLL.srv` — Service definitions
- `launch/geo_transformer.launch.py` — Launch file
- `scripts/test_services.py` — Python test script
- `CMakeLists.txt`, `package.xml` — Build and package configuration
- `README.md`, `DOCUMENTATION.md` — Documentation

---

## License
This package is released under the MIT License.

---

## Contact
For questions or contributions, please contact the package maintainer listed in `package.xml`.



