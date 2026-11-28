# OSM + OSMnx + A* + ROS 2 Jazzy + RViz2

A semester-project prototype for selecting two addresses in Oakland County, Michigan, downloading the county road network from OpenStreetMap, finding a shortest-distance drivable route with A*, and visualizing the road graph, route and waypoints in RViz2.

## 1. Target platform

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3.12
- PyQt5 GUI
- OSMnx 2.x, NetworkX, PyProj and Shapely
- RViz2

This package is for **global road-network path planning and visualization**. The Audibot car model is animated along the planned route with a kinematic pure pursuit follower in RViz2; there is no physics simulation, obstacle avoidance, localization or Nav2 control.

## 2. Install ROS 2 Jazzy

Follow the current official ROS 2 Jazzy installation guide for Ubuntu 24.04:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Install the desktop variant so RViz2 is included. Then source ROS in each terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

To source automatically in new terminals:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 3. Install project dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-pyqt5 \
  python3-pyproj \
  python3-networkx \
  python3-shapely \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher
```

Install OSMnx into the Python environment used by ROS 2. On Ubuntu 24.04, pip may enforce the externally-managed-environment rule. For a dedicated robotics development machine, this command installs it for the current user:

```bash
python3 -m pip install --user --break-system-packages -r requirements.txt
```

If pip is not installed:

```bash
sudo apt install -y python3-pip
```

Check imports:

```bash
python3 -c "import osmnx, networkx, pyproj, shapely, PyQt5; print('Python dependencies OK')"
```

## 4. Create workspace and build

```bash
mkdir -p ~/ros2_ws/src
```

Place the ROS package inside ros2_ws/src:

`~/ros2_ws/src/osm_robot_navigation`

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add the workspace overlay to new terminals:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

## 5. Run the complete application

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch osm_robot_navigation navigation.launch.py
```

The launch file starts:

1. `route_planner` - subscribes to `/route_request` and calculates A*.
2. `rviz2` - displays the OSM road network, path, markers and the Audibot vehicle model.
3. `location_gui` - PyQt5 form to enter the start and destination.
4. `robot_state_publisher` - expands `urdf/audibot_description.urdf` with xacro and publishes `/robot_description` and the vehicle TF tree.
5. `path_follower` - subscribes to `/planned_path` and drives the Audibot along it with pure pursuit on a kinematic bicycle model. It publishes the `map -> base_footprint` transform and the wheel/steering `/joint_states`. Before the first route the car sits at the `map` origin (the county centroid).

Launch options:

| Argument | Default | Meaning |
|---|---|---|
| `blue` | `true` | Blue body (`false` = orange) |
| `speed` | `15.0` | Cruise speed in m/s; the car slows for curves and the goal |
| `time_scale` | `1.0` | Playback speed-up (e.g. `10.0` drives a 20 km route in ~2 min) |

```bash
ros2 launch osm_robot_navigation navigation.launch.py blue:=false speed:=20.0 time_scale:=10.0
```

The RViz camera follows `base_footprint` from ~60 m away. Scroll out to see the whole road network.

The GUI starts with empty start and destination fields. Enter both before submitting.

## 6. ROS interfaces

| Topic | Type | Purpose |
|---|---|---|
| `/route_request` | `std_msgs/msg/String` | JSON request: `{"start":"Troy, Michigan","goal":"Rochester Hills, Michigan"}` |
| `/route_status` | `std_msgs/msg/String` | Progress, success or error message |
| `/planned_path` | `nav_msgs/msg/Path` | Ordered A* route poses |
| `/road_network` | `visualization_msgs/msg/Marker` | Sampled OSM road network as line segments |
| `/route_line` | `visualization_msgs/msg/Marker` | Planned route drawn as an orange line above the road network |
| `/route_markers` | `visualization_msgs/msg/MarkerArray` | Start, goal and intermediate waypoints |
| `/joint_states` | `sensor_msgs/msg/JointState` | Audibot wheel spin and steering angles from `path_follower` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map -> base_footprint` from `path_follower`, vehicle links from `robot_state_publisher` |

Inspect topics:

```bash
ros2 topic list
ros2 topic echo /route_status
ros2 topic echo /planned_path --once
```

## 7. How the algorithm works

- Nominatim geocodes the entered addresses to latitude/longitude.
- OSMnx obtains the Oakland County boundary and downloads the drivable road graph.
- The first graph download is cached at `~/.cache/osm_robot_navigation/oakland_county_drive.graphml`.
- OSMnx finds the nearest graph nodes to the two geocoded locations.
- A* searches the directed road graph. Edge costs are OSM road lengths in meters; the heuristic is straight-line haversine distance in meters.
- The route node coordinates are projected to UTM zone 17N, then shifted relative to the Oakland County centroid to create a local `map` frame suitable for RViz.
- The node publishes the path and visualization markers.
- `path_follower` tracks the path with pure pursuit: it picks a point `max(4 m, 0.3 s x speed)` ahead on the route, steers with `delta = atan(2 L sin(alpha) / Ld)` (L = 2.652 m wheelbase, clamped to +/-0.514 rad so the inner front wheel stays within its 0.6 rad limit), limits speed to keep lateral acceleration under 3 m/s^2, and brakes before the goal. Front wheel angles use Ackermann geometry.


<img width="840" height="1848" alt="OSM_Robot_Navigation_Flowchart" src="https://github.com/user-attachments/assets/80ff67d4-74e2-4346-828e-28d3fe739431" />