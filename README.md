# geo_transformer ROS 2 Package

## Overview
This package provides ROS 2 services for transforming geodetic coordinates (latitude, longitude, altitude) to local coordinates (x, y, z) and vice versa.

## Prerequisites
- ROS 2 (e.g., Foxy, Galactic, Humble, etc.) installed and sourced
- Colcon build system

## Building the Package
1. Open a terminal and navigate to the workspace root:
   ```bash
   cd /home/sahil/Desktop/The_assignment/asg_ws
   ```
2. Build the workspace:
   ```bash
   colcon build --packages-select geo_transformer
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Node
To run the main node (e.g., `set_origin_node`):
```bash
ros2 run geo_transformer set_origin_node
```

## Calling the Service
Assuming you want to call the `FromLL` service (convert latitude, longitude, altitude to x, y, z):

1. In a new terminal, source your workspace:
   ```bash
   cd /home/sahil/Desktop/The_assignment/asg_ws
   source install/setup.bash
   ```
2. Call the service using `ros2 service call`:
   ```bash
   ros2 service call /from_ll geo_transformer/srv/FromLL "{latitude: 12.34, longitude: 56.78, altitude: 100.0}"
   ```
   Replace the values with your desired coordinates.

## Example Output
```
x: 123.45
y: 678.90
z: 50.0
success: true
message: "Transformation successful."
```

## Notes
- Make sure the node is running before calling the service.
- You can check available services with:
  ```bash
  ros2 service list
  ```
- You can inspect the service type with:
  ```bash
  ros2 interface show geo_transformer/srv/FromLL
  ```
