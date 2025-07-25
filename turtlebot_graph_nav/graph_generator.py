#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, ColorRGBA
import json
import math
import numpy as np
from typing import Dict, List, Tuple, Optional


class GraphGenerator(Node):
    def __init__(self):
        super().__init__('graph_generator')
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, 'graph_markers', 10)
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        
        # Parameters
        self.declare_parameter('map_file', 'config/map_graph.json')
        self.declare_parameter('frame_id', 'map')
        
        # Load map data
        self.map_data = self.load_map_data()
        self.graph = self.build_graph()
        
        # Timer for visualization
        self.timer = self.create_timer(1.0, self.publish_graph_markers)
        
        self.get_logger().info('Graph Generator Node initialized')

    def load_map_data(self) -> Dict:
        """Load map structure from JSON file"""
        try:
            map_file = self.get_parameter('map_file').value
            with open(map_file, 'r') as f:
                data = json.load(f)
            
            self.get_logger().info(f'Loaded map data from {map_file}')
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load map data: {e}')
            # Return sample data structure
            return self.get_sample_map_data()

    def get_sample_map_data(self) -> Dict:
        """Sample map data structure"""
        return {
            "nodes": [
                {"id": "A", "x": 0.0, "y": 0.0, "type": "waypoint"},
                {"id": "B", "x": 5.0, "y": 0.0, "type": "waypoint"},
                {"id": "C", "x": 5.0, "y": 5.0, "type": "waypoint"},
                {"id": "D", "x": 0.0, "y": 5.0, "type": "waypoint"},
                {"id": "E", "x": 2.5, "y": 2.5, "type": "intersection"}
            ],
            "edges": [
                {"from": "A", "to": "B", "weight": 5.0, "bidirectional": True},
                {"from": "B", "to": "C", "weight": 5.0, "bidirectional": True},
                {"from": "C", "to": "D", "weight": 5.0, "bidirectional": True},
                {"from": "D", "to": "A", "weight": 5.0, "bidirectional": True},
                {"from": "A", "to": "E", "weight": 3.54, "bidirectional": True},
                {"from": "B", "to": "E", "weight": 3.54, "bidirectional": True},
                {"from": "C", "to": "E", "weight": 3.54, "bidirectional": True},
                {"from": "D", "to": "E", "weight": 3.54, "bidirectional": True}
            ]
        }

    def build_graph(self) -> Dict:
        """Build graph structure from map data"""
        graph = {}
        
        # Create nodes dictionary for quick lookup
        nodes_dict = {node['id']: node for node in self.map_data['nodes']}
        
        # Initialize graph with all nodes
        for node in self.map_data['nodes']:
            graph[node['id']] = {
                'position': (node['x'], node['y']),
                'type': node.get('type', 'waypoint'),
                'neighbors': []
            }
        
        # Add edges
        for edge in self.map_data['edges']:
            from_node = edge['from']
            to_node = edge['to']
            weight = edge.get('weight', self.calculate_distance(
                nodes_dict[from_node], nodes_dict[to_node]
            ))
            
            graph[from_node]['neighbors'].append({
                'node': to_node,
                'weight': weight
            })
            
            # Add reverse edge if bidirectional
            if edge.get('bidirectional', True):
                graph[to_node]['neighbors'].append({
                    'node': from_node,
                    'weight': weight
                })
        
        self.get_logger().info(f'Built graph with {len(graph)} nodes')
        return graph

    def calculate_distance(self, node1: Dict, node2: Dict) -> float:
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((node1['x'] - node2['x'])**2 + (node1['y'] - node2['y'])**2)

    def publish_graph_markers(self):
        """Publish visualization markers for the graph"""
        marker_array = MarkerArray()
        frame_id = self.get_parameter('frame_id').value
        
        # Node markers
        for i, (node_id, node_data) in enumerate(self.graph.items()):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = node_data['position'][0]
            marker.pose.position.y = node_data['position'][1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color based on node type
            if node_data['type'] == 'intersection':
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            else:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            
            marker_array.markers.append(marker)
        
        # Edge markers
        edge_id = len(self.graph)
        for node_id, node_data in self.graph.items():
            for neighbor in node_data['neighbors']:
                neighbor_pos = self.graph[neighbor['node']]['position']
                
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "edges"
                marker.id = edge_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # Start point
                start_point = Point()
                start_point.x = node_data['position'][0]
                start_point.y = node_data['position'][1]
                start_point.z = 0.05
                
                # End point
                end_point = Point()
                end_point.x = neighbor_pos[0]
                end_point.y = neighbor_pos[1]
                end_point.z = 0.05
                
                marker.points = [start_point, end_point]
                
                marker.scale.x = 0.05  # Line width
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue
                
                marker_array.markers.append(marker)
                edge_id += 1
        
        # Text markers for node IDs
        text_id = edge_id
        for node_id, node_data in self.graph.items():
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "text"
            marker.id = text_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = node_data['position'][0]
            marker.pose.position.y = node_data['position'][1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.3  # Text height
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            marker.text = node_id
            
            marker_array.markers.append(marker)
            text_id += 1
        
        self.marker_pub.publish(marker_array)

    def dijkstra(self, start: str, goal: str) -> Optional[List[str]]:
        """Dijkstra's algorithm for shortest path finding"""
        if start not in self.graph or goal not in self.graph:
            return None
        
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0
        previous = {}
        unvisited = set(self.graph.keys())
        
        while unvisited:
            current = min(unvisited, key=lambda node: distances[node])
            unvisited.remove(current)
            
            if current == goal:
                break
            
            for neighbor_info in self.graph[current]['neighbors']:
                neighbor = neighbor_info['node']
                weight = neighbor_info['weight']
                
                if neighbor in unvisited:
                    new_distance = distances[current] + weight
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        previous[neighbor] = current
        
        # Reconstruct path
        if goal not in previous and start != goal:
            return None
        
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        return path[::-1]

    def publish_path(self, start: str, goal: str):
        """Find and publish path between two nodes"""
        path_nodes = self.dijkstra(start, goal)
        
        if not path_nodes:
            self.get_logger().warn(f'No path found from {start} to {goal}')
            return
        
        path_msg = Path()
        path_msg.header.frame_id = self.get_parameter('frame_id').value
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for node_id in path_nodes:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            pos = self.graph[node_id]['position']
            pose_stamped.pose.position.x = pos[0]
            pose_stamped.pose.position.y = pos[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path: {" -> ".join(path_nodes)}')

    def get_graph_info(self) -> Dict:
        """Get graph information for external use"""
        return {
            'nodes': list(self.graph.keys()),
            'graph': self.graph
        }


def main(args=None):
    rclpy.init(args=args)
    
    graph_generator = GraphGenerator()
    
    try:
        rclpy.spin(graph_generator)
    except KeyboardInterrupt:
        pass
    finally:
        graph_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()