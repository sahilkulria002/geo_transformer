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
1. Open **Terminal 3** and install Python dependencies (only needed once):
   ```bash
   cd /home/asg_ws
   pip install -r scripts/requirements.txt
   ```
2. Source your workspace:
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

### 3. FromLL Service (Geodetic to Local, Batch Supported)
```bash
# Single point (must use arrays):
ros2 service call /from_ll geo_transformer/srv/FromLL "{latitude: [28.6139], longitude: [77.20900898], altitude: [0.0]}"
# Batch (multiple points, ~1m apart in x, y, z):
ros2 service call /from_ll geo_transformer/srv/FromLL "{latitude: [28.6139, 28.61390898, 28.6139], longitude: [77.20900898, 77.2090, 77.2090], altitude: [0.0, 0.0, 1.0]}"
```

Example output (single point, ~1m east of origin):
```
x: [1.0001]
y: [0.0]
z: [0.0]
success: true
message: "All transformations successful. | Origin name: 'my_origin', lat: 28.6139, lon: 77.2090, alt: 0.0"
```
Example output (batch, points ~1m apart in x, y, z):
```
x: [1.0001, 0.0, 0.0]
y: [0.0, 1.0001, 0.0]
z: [0.0, 0.0, 1.0]
success: true
message: "All transformations successful. | Origin name: 'my_origin', lat: 28.6139, lon: 77.2090, alt: 0.0"
```

### 4. ToLL Service (Local to Geodetic, Batch Supported)
```bash
# Single point (must use arrays):
ros2 service call /to_ll geo_transformer/srv/ToLL "{x: [1.0], y: [0.0], z: [0.0]}"
# Batch (multiple points):
ros2 service call /to_ll geo_transformer/srv/ToLL "{x: [1.0, 0.0, 0.0], y: [0.0, 1.0, 0.0], z: [0.0, 0.0, 1.0]}"
```
Example output (single point):
```
latitude: [28.6139]
longitude: [77.20900898]
altitude: [0.0]
success: true
message: "All transformations successful. | Origin name: 'my_origin', lat: 28.6139, lon: 77.2090, alt: 0.0"
```
Example output (batch):
```
latitude: [28.6139, 28.61390898, 28.6139]
longitude: [77.20900898, 77.2090, 77.2090]
altitude: [0.0, 0.0, 1.0]
success: true
message: "All transformations successful. | Origin name: 'my_origin', lat: 28.6139, lon: 77.2090, alt: 0.0"
```
## ################################################
## Advanced: Named Origin Management

The following additional services allow you to manage and switch between multiple named origins, without affecting the default single-origin workflow:

### 1. Add a Named Origin
```bash
ros2 service call /origin_store/add_named_origin geo_transformer/srv/AddNamedOrigin '{name: "my_origin", latitude: 28.6139, longitude: 77.2090, altitude: 0.0}'
```

### 2. Switch to a Named Origin
```bash
ros2 service call /origin_store/switch_origin geo_transformer/srv/SwitchOrigin '{name: "my_origin"}'
```

### 3. List All Named Origins
```bash
ros2 service call /origin_store/list_origins geo_transformer/srv/ListOrigins '{}'
```

### 4. Get Current Origin Name
```bash
ros2 service call /origin_store/get_current_origin_name geo_transformer/srv/GetCurrentOriginName '{}'
```

---

## Visualization and Node Graph

### Visualizing Transformed Points and Origin in RViz

You can visualize the transformed points and the origin in RViz using the provided Python visualization node:

1. **Start the visualization node:**
   ```bash
   python3 geo_transformer/scripts/visualize_points.py
   ```
2. **Start RViz:**
   ```bash
   rviz2
   ```
3. **In RViz:**
   - Set the Fixed Frame to `odom`.
   - Add a `MarkerArray` display and set the topic to `/geo_transformer/markers`.
   - Add a `Grid` and `Axes` display for reference.
   - Use the selection tool to inspect marker coordinates.

The origin will appear as a red sphere labeled "Origin". Transformed points will appear as green spheres with labels (P0, P1, ...), spaced ~1m apart in x, y, z from the origin.

### Node Graph

To see the ROS 2 node and topic graph, run:
```bash
ros2 run rqt_graph rqt_graph
```
This will show all nodes and their topic connections, including services and publishers/subscribers.

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
  ros2 interface show geo_transformer/srv/AddNamedOrigin
  ros2 interface show geo_transformer/srv/SwitchOrigin
  ros2 interface show geo_transformer/srv/ListOrigins
  ros2 interface show geo_transformer/srv/GetCurrentOriginName
  ```
