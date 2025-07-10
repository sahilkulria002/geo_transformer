# geo_transformer ROS 2 Package

## Overview
This package provides ROS 2 services for transforming geodetic coordinates (latitude, longitude, altitude) to local coordinates (x, y, z) and vice versa.
This package uses [GeographicLib](https://geographiclib.sourceforge.io/) for geodetic ↔ local coordinate conversions.


## Prerequisites
- ROS 2 (e.g., Foxy, Galactic, Humble, etc.) installed and sourced
- Colcon build system
- Python 3 (for test scripts)
- Python dependencies for test scripts:
  ```bash
  pip install -r scripts/requirements.txt
  ```


## Building the Package (Terminal 1)
1. Open **Terminal 1** and navigate to the workspace root:
   ```bash
   cd /home/asg_ws
   ```
2. Build the workspace (this will compile all C++ and generate Python interfaces):
   ```bash
   colcon build --packages-select geo_transformer
   ```
3. Source the workspace (must be done in every terminal before using ROS 2 commands):
   ```bash
   source install/setup.bash
   ```



## Running the Node (Terminal 2)
You can run the node using either a launch file or directly:

### Option 1: Using the launch file (recommended)
Open **Terminal 2**:
```bash
cd /home/asg_ws
source install/setup.bash
ros2 launch geo_transformer geo_transformer.launch.py
```

### Option 2: Run the node directly
Open **Terminal 2**:
```bash
cd /home/asg_ws
source install/setup.bash
ros2 run geo_transformer geo_transformer_node
```

---

## Testing with the Provided Python Script (Terminal 3)
1. Open **Terminal 3**  and source your workspace:
   ```bash
   source install/setup.bash
   ```
3. Run the test script:
   ```bash
   ros2 run geo_transformer test_services.py
   ```
   This will automatically:
   - Set the origin (India Gate)
   - Get the origin
   - Convert a test WGS84 point to local coordinates
   - Convert back to WGS84

---


### Integration with Other Nodes

Once launched, this node exposes 4 ROS2 services which can be called from:
- Command line using `ros2 service call`
- Any other ROS2 node in C++ or Python
- Automated launch and test pipelines

The service interface is decoupled and clean — no custom dependencies, so it plugs easily into mapping, localization, or robot control pipelines.

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

## ###############################################################################
## Calling Services Manually (Any Terminal)

You can call the services from any terminal (after sourcing the workspace):

### 1. Set the Origin
```bash
ros2 service call /local_coordinate/set geo_transformer/srv/SetOrigin "{latitude: 40.6892, longitude: -74.0445, altitude: 0.0}"
```
Example output:
```
success: true
message: "Origin set to: 40.689200, -74.044500, 0.000000"
```

### 2. Get the Origin
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

### 3. FromLL Service (Geodetic to Local)
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

### 4. ToLL Service (Local to Geodetic)
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


  ```
