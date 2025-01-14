#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import pyproj
import json
import os
from ament_index_python.packages import get_package_share_directory
import argparse

class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.declare_parameter('coords_file', '')
        self.coords_file = self.get_parameter('coords_file').get_parameter_value().string_value
        if not self.coords_file:
            self.get_logger().error('No coordinates file provided')
            return
        self.process_coordinates()

    def utm_to_gps(self, utm_proj, utm_x, utm_y):
        gps_proj = pyproj.Proj(proj='latlong', datum='WGS84')
        lon, lat = pyproj.transform(utm_proj, gps_proj, utm_x, utm_y)
        return lat, lon

    def save_waypoints(self, boustrophedon_path, utm_proj):
        lap_waypoints = []
        for point in boustrophedon_path:
            lat, lon = self.utm_to_gps(utm_proj, point[0], point[1])
            waypoint = {
                "latitude": round(lat, 7),
                "longitude": round(lon, 7),
                "altitude": 90
            }
            lap_waypoints.append(waypoint)
        
        waypoint_data = {
            "lap_waypoints": lap_waypoints,
            "coverage_waypoints": [],
            "airdrop_waypoints": []
        }

        package_share_dir = get_package_share_directory("Coverage_Planner")
        waypoints_file = os.path.join(package_share_dir, 'Waypoints', 'coverage_waypoints.json')
        os.makedirs(os.path.dirname(waypoints_file), exist_ok=True)

        with open(waypoints_file, 'w') as f:
            json.dump(waypoint_data, f, indent=4)
        
        self.get_logger().info(f"Saved {len(lap_waypoints)} waypoints to {waypoints_file}")

    def process_coordinates(self):
        with open(self.coords_file, 'r') as f:
            data = json.load(f)
            latitudes = [wp["latitude"] for wp in data.get("lap_waypoints",[])]
            longitudes = [wp["longitude"] for wp in data.get("lap_waypoints",[])]
        
        points = list(zip(longitudes, latitudes))
        utm_proj = pyproj.Proj(proj='utm', zone=18, ellps='WGS84')
        utm_points = [utm_proj(lon, lat) for lon, lat in points]
        polygon = Polygon(utm_points)
        
        cell_width = 5
        cell_height = 5
        centroids = []
        min_x, min_y, max_x, max_y = polygon.bounds
        cells = []
        x = min_x
        while x < max_x:
            y = min_y
            while y < max_y:
                cell = Polygon([(x, y), (x + cell_width, y), 
                              (x + cell_width, y + cell_height), 
                              (x, y + cell_height)])
                intersection = polygon.intersection(cell)
                if not intersection.is_empty and not isinstance(intersection, Point):
                    cells.append(cell)
                    centroids.append(cell.centroid)
                y += cell_height
            x += cell_width
        
        centroids_within_polygon = [centroid for centroid in centroids 
                                  if polygon.contains(centroid)]
        centroid_coords = np.array([(c.x, c.y) for c in centroids_within_polygon])
        sorted_centroids = sorted(centroid_coords, key=lambda p: (p[1], p[0]))
        
        rows = []
        current_row = []
        row_y = sorted_centroids[0][1]
        tolerance = 1e-3
        
        for centroid in sorted_centroids:
            if abs(centroid[1] - row_y) < tolerance:
                current_row.append(centroid)
            else:
                rows.append(current_row)
                current_row = [centroid]
                row_y = centroid[1]
        rows.append(current_row)
        
        boustrophedon_path = []
        for i, row in enumerate(rows):
            if i % 2 == 0:
                boustrophedon_path.extend(sorted(row, key=lambda p: p[0]))
            else:
                boustrophedon_path.extend(sorted(row, key=lambda p: p[0], 
                                                reverse=True))
        
        fig, ax = plt.subplots()
        x, y = polygon.exterior.xy
        ax.plot(x, y, color='red', label='Polygon Boundary')
        
        for centroid in centroids_within_polygon:
            ax.plot(centroid.x, centroid.y, 'bo')
        
        path_x, path_y = zip(*boustrophedon_path)
        ax.plot(path_x, path_y, 'g-', marker='o', label='Boustrophedon Path')
        ax.set_aspect('equal')
        plt.legend()
        plt.show()
        
        self.get_logger().info("Boustrophedon Path:")
        for point in boustrophedon_path:
            self.get_logger().info(f"({point[0]}, {point[1]})")
        self.get_logger().info(f"Total waypoints: {len(boustrophedon_path)}")
        
        self.save_waypoints(boustrophedon_path, utm_proj)

def main():
    rclpy.init()
    node = CoveragePlannerNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()