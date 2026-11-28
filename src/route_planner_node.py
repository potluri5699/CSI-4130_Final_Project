#!/usr/bin/env python3
"""ROS 2 node: geocode addresses, load Oakland County roads, run A*, publish RViz markers."""

import json
import math
import os
import pathlib
import traceback
import heapq
import itertools

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Transformer

try:
    import osmnx as ox
except ImportError as exc:
    ox = None
    OSMNX_IMPORT_ERROR = str(exc)
else:
    OSMNX_IMPORT_ERROR = None

try:
    from shapely.geometry import Point as ShapelyPoint
except ImportError:
    ShapelyPoint = None


PLACE = "Oakland County, Michigan, USA"
FRAME_ID = "map"
UTM_EPSG = "EPSG:32617"  # Oakland County is in UTM zone 17N.
CACHE_DIR = pathlib.Path.home() / ".cache" / "osm_robot_navigation"
GRAPH_FILE = CACHE_DIR / "oakland_county_drive.graphml"
MAX_ROAD_SEGMENTS = 25000
MAX_WAYPOINT_MARKERS = 500


def haversine_m(lon1, lat1, lon2, lat2):
    """Great-circle distance in meters; admissible for distance-weighted A*."""
    radius = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 2.0 * radius * math.asin(min(1.0, math.sqrt(a)))


def astar_road_graph(graph, start, goal):
    """A* over a directed OSMnx MultiDiGraph using edge length in meters.

    Returns (route, distance_m, expanded, edges) where edges[i] is the shortest edge
    data dict between route[i] and route[i + 1]; it carries the curved "geometry".
    """
    adjacency = {}
    for u, v, data in graph.edges(data=True):
        length = float(data.get("length", 1.0))
        if length < 0:
            continue
        old = adjacency.setdefault(u, {}).get(v)
        if old is None or length < old[0]:
            adjacency[u][v] = (length, data)

    def heuristic(node):
        a, b = graph.nodes[node], graph.nodes[goal]
        return haversine_m(float(a["x"]), float(a["y"]), float(b["x"]), float(b["y"]))

    counter = itertools.count()
    frontier = [(heuristic(start), next(counter), start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if current == goal:
            route = [current]
            while current in came_from:
                current = came_from[current]
                route.append(current)
            route.reverse()
            edges = [adjacency[a][b][1] for a, b in zip(route, route[1:])]
            return route, g_score[goal], len(closed) + 1, edges

        closed.add(current)
        for neighbor, (edge_length, _) in adjacency.get(current, {}).items():
            tentative = g_score[current] + edge_length
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(frontier, (tentative + heuristic(neighbor), next(counter), neighbor))

    raise RuntimeError("No drivable route was found between the selected locations.")


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("osm_astar_route_planner")
        self.request_sub = self.create_subscription(String, "/route_request", self.request_callback, 10)
        self.status_pub = self.create_publisher(String, "/route_status", 10)
        self.path_pub = self.create_publisher(
            Path, "/planned_path",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_pub = self.create_publisher(
            Marker, "/road_network",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.route_line_pub = self.create_publisher(
            Marker, "/route_line",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.markers_pub = self.create_publisher(MarkerArray, "/route_markers", 10)
        self.graph = None
        self.road_marker = None
        self.boundary = None
        self.transformer = Transformer.from_crs("EPSG:4326", UTM_EPSG, always_xy=True)
        self.origin_x = None
        self.origin_y = None
        self.get_logger().info("A* route planner ready. Waiting on /route_request (JSON String).")
        if OSMNX_IMPORT_ERROR:
            self.get_logger().error("OSMnx import failed: " + OSMNX_IMPORT_ERROR)

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(message)

    def load_oakland_graph(self):
        if ox is None:
            raise RuntimeError("OSMnx is not installed.")
        if self.graph is not None:
            return

        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        self.publish_status("Loading Oakland County boundary and road network...")
        ox.settings.use_cache = True
        ox.settings.log_console = False
        ox.settings.requests_timeout = 180

        boundary_gdf = ox.geocode_to_gdf(PLACE)
        self.boundary = boundary_gdf.geometry.union_all()
        centroid = self.boundary.centroid
        self.origin_x, self.origin_y = self.transformer.transform(centroid.x, centroid.y)

        if GRAPH_FILE.exists():
            self.get_logger().info("Loading cached road graph: %s" % GRAPH_FILE)
            self.graph = ox.load_graphml(filepath=str(GRAPH_FILE))
        else:
            self.get_logger().info("Downloading drivable road network for %s" % PLACE)
            self.graph = ox.graph_from_polygon(self.boundary, network_type="drive", simplify=True, retain_all=False)
            ox.save_graphml(self.graph, filepath=str(GRAPH_FILE))
            self.get_logger().info("Saved road graph cache: %s" % GRAPH_FILE)

    def publish_cached_road_network(self):
        """Publish the road network marker, rebuilding it once and refreshing its stamp on later requests."""
        if self.road_marker is None:
            self.publish_road_network()
            return
        self.road_marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.road_marker)

    def publish_road_network(self):
        """Publish a bounded sample of the OSM road graph as a Marker LINE_LIST."""
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "osm_roads"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.65
        marker.color = ColorRGBA(r=0.48, g=0.52, b=0.58, a=0.55)
        seen = set()
        count = 0
        for u, v, data in self.graph.edges(data=True):
            key = (min(u, v), max(u, v))
            if key in seen:
                continue
            seen.add(key)
            if "geometry" in data:
                coords = list(data["geometry"].coords)
            else:
                nu, nv = self.graph.nodes[u], self.graph.nodes[v]
                coords = [(nu["x"], nu["y"]), (nv["x"], nv["y"])]
            if len(coords) < 2:
                continue
            # Draw each geometry as connected line segments.
            for p1, p2 in zip(coords[:-1], coords[1:]):
                x1, y1 = self.transformer.transform(p1[0], p1[1])
                x2, y2 = self.transformer.transform(p2[0], p2[1])
                a, b = Point(), Point()
                a.x, a.y, a.z = x1 - self.origin_x, y1 - self.origin_y, 0.03
                b.x, b.y, b.z = x2 - self.origin_x, y2 - self.origin_y, 0.03
                marker.points.extend([a, b])
            count += 1
            if count >= MAX_ROAD_SEGMENTS:
                break
        self.road_marker = marker
        self.marker_pub.publish(marker)
        self.get_logger().info("Published %d road segments to /road_network" % count)

    def request_callback(self, msg):
        try:
            if ox is None:
                raise RuntimeError("OSMnx import failed.")
            request = json.loads(msg.data)
            start_address = request.get("start", "").strip()
            goal_address = request.get("goal", "").strip()
            if not start_address or not goal_address:
                raise ValueError("Both start and destination addresses are required.")
            self.publish_status("Geocoding start and destination...")
            start_lat, start_lon = ox.geocode(start_address)
            goal_lat, goal_lon = ox.geocode(goal_address)
            self.load_oakland_graph()
            self.publish_cached_road_network()

            if ShapelyPoint is not None:
                if not self.boundary.covers(ShapelyPoint(start_lon, start_lat)):
                    raise ValueError("Start location is outside Oakland County, Michigan.")
                if not self.boundary.covers(ShapelyPoint(goal_lon, goal_lat)):
                    raise ValueError("Destination is outside Oakland County, Michigan.")

            start_node = ox.distance.nearest_nodes(self.graph, X=start_lon, Y=start_lat)
            goal_node = ox.distance.nearest_nodes(self.graph, X=goal_lon, Y=goal_lat)
            self.publish_status("Running A* shortest-distance search...")
            route, distance_m, expanded, edges = astar_road_graph(self.graph, start_node, goal_node)
            self.publish_route(route, edges)
            self.publish_status("SUCCESS | Route: %.2f km | Nodes in route: %d | A* expanded: %d" %
                                (distance_m / 1000.0, len(route), expanded))
        except Exception as exc:
            self.get_logger().error("Route request failed: %s\n%s" % (exc, traceback.format_exc()))
            self.publish_status("ERROR | " + str(exc))

    def lonlat_xy(self, lon, lat):
        east, north = self.transformer.transform(float(lon), float(lat))
        return east - self.origin_x, north - self.origin_y

    def node_xy(self, node_id):
        node = self.graph.nodes[node_id]
        return self.lonlat_xy(node["x"], node["y"])

    def route_xy(self, route, edges):
        """Dense map-frame polyline: each road's curve geometry between its two intersections."""
        xy = [self.node_xy(route[0])]
        for node_id, data in zip(route[1:], edges):
            if "geometry" in data:
                # OSMnx stores edge geometry from u to v; the endpoints equal the nodes, so skip them.
                xy.extend(self.lonlat_xy(lon, lat) for lon, lat in list(data["geometry"].coords)[1:-1])
            xy.append(self.node_xy(node_id))
        deduped = [xy[0]]
        for x, y in xy[1:]:
            if math.hypot(x - deduped[-1][0], y - deduped[-1][1]) > 1e-3:
                deduped.append((x, y))
        return deduped

    def publish_route(self, route, edges):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = FRAME_ID
        path.header.stamp = now
        xy = self.route_xy(route, edges)

        for index, (x, y) in enumerate(xy):
            pose = PoseStamped()
            pose.header.frame_id = FRAME_ID
            pose.header.stamp = now
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.12
            if index < len(xy) - 1:
                dx, dy = xy[index + 1][0] - x, xy[index + 1][1] - y
            elif index > 0:
                dx, dy = x - xy[index - 1][0], y - xy[index - 1][1]
            else:
                dx, dy = 1.0, 0.0
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self.path_pub.publish(path)

        line = Marker()
        line.header.frame_id, line.header.stamp = FRAME_ID, now
        line.ns, line.id = "planned_route", 0
        line.type, line.action = Marker.LINE_STRIP, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 2.0
        line.color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
        line.points = [Point(x=x, y=y, z=0.12) for x, y in xy]
        self.route_line_pub.publish(line)

        array = MarkerArray()
        start = Marker()
        start.header.frame_id, start.header.stamp = FRAME_ID, now
        start.ns, start.id, start.type, start.action = "route_start", 1, Marker.SPHERE, Marker.ADD
        start.pose.position.x, start.pose.position.y, start.pose.position.z = xy[0][0], xy[0][1], 0.35
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = start.scale.z = 5.0
        start.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        goal = Marker()
        goal.header.frame_id, goal.header.stamp = FRAME_ID, now
        goal.ns, goal.id, goal.type, goal.action = "route_goal", 2, Marker.SPHERE, Marker.ADD
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = xy[-1][0], xy[-1][1], 0.35
        goal.pose.orientation.w = 1.0
        goal.scale.x = goal.scale.y = goal.scale.z = 5.0
        goal.color = ColorRGBA(r=0.95, g=0.12, b=0.12, a=1.0)

        waypoints = Marker()
        waypoints.header.frame_id, waypoints.header.stamp = FRAME_ID, now
        waypoints.ns, waypoints.id = "route_waypoints", 3
        waypoints.type, waypoints.action = Marker.SPHERE_LIST, Marker.ADD
        waypoints.pose.orientation.w = 1.0
        waypoints.scale.x = waypoints.scale.y = waypoints.scale.z = 1.8
        waypoints.color = ColorRGBA(r=0.1, g=0.55, b=1.0, a=0.9)
        stride = max(1, len(xy) // MAX_WAYPOINT_MARKERS)
        for x, y in xy[::stride]:
            p = Point()
            p.x, p.y, p.z = x, y, 0.25
            waypoints.points.append(p)
        array.markers = [start, goal, waypoints]
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
