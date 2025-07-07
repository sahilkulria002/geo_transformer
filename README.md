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
To run the main node:
```bash
ros2 run geo_transformer geo_transformer_node
```

## Calling the Services


### 1. Set the Origin
You must set the origin before using the other services:
```bash
ros2 service call /local_coordinate/set geo_transformer/srv/SetOrigin "{latitude: 40.6892, longitude: -74.0445, altitude: 0.0}"
```
Example output:
```
success: true
message: "Origin set to: 40.689200, -74.044500, 0.000000"
```

### 1b. Get the Origin
You can get the current origin:
```bash
ros2 service call /local_coordinate/get geo_transformer/srv/GetOrigin "{}"
```
Example output:
```
latitude: 40.6892
longitude: -74.0445
altitude: 0.0
success: true
message: "Origin retrieved successfully."
```


### 2. FromLL Service (Geodetic to Local)
Convert latitude, longitude, altitude to x, y, z:
```bash
ros2 service call /from_ll geo_transformer/srv/FromLL "{latitude: 40.6900, longitude: -74.0450, altitude: 10.0}"
```
Example output:
```
x: 70.5
y: -89.1
z: 10.0
success: true
message: "Transformation successful."
```


### 3. ToLL Service (Local to Geodetic)
Convert x, y, z to latitude, longitude, altitude:
```bash
ros2 service call /to_ll geo_transformer/srv/ToLL "{x: 70.5, y: -89.1, z: 10.0}"
```
Example output:
```
latitude: 40.6900
longitude: -74.0450
altitude: 10.0
success: true
message: "Transformation successful."
```

## Notes
- Make sure the node is running before calling the services.
- You can check available services with:
  ```bash
  ros2 service list
  ```
- You can inspect the service types with:
  ```bash
  ros2 interface show geo_transformer/srv/FromLL
  ros2 interface show geo_transformer/srv/ToLL
  ros2 interface show geo_transformer/srv/SetOrigin
  ros2 interface show geo_transformer/srv/GetOrigin
  ```
