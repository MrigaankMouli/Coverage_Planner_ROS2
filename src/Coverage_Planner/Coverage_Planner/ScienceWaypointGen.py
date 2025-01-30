import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer
import math
import json
import os
from ament_index_python.packages import get_package_share_directory

def utm_to_gps(utm_proj, utm_x, utm_y):
    gps_proj = Proj(proj='latlong', datum='WGS84')
    transformer = Transformer.from_proj(utm_proj, gps_proj, always_xy=True)
    lon, lat = transformer.transform(utm_x, utm_y)
    return lat, lon

def gps_to_utm(lon, lat):
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    transformer = Transformer.from_proj(Proj(proj='latlong', datum='WGS84'), utm_proj, always_xy=True)
    utm_x, utm_y = transformer.transform(lon, lat)
    return utm_x, utm_y

def generate_sequential_paths(waypoints, heights, fov_horizontal=90, fov_vertical=65):
    all_paths = []
    all_centers = []

    reference_height_meters = heights[0] * 0.3048
    horizontal_fov_radians = math.radians(fov_horizontal)
    vertical_fov_radians = math.radians(fov_vertical)
    
    reference_half_sq_side_x = reference_height_meters * math.tan(horizontal_fov_radians / 2)
    reference_half_sq_side_y = reference_height_meters * math.tan(vertical_fov_radians / 2)

    for waypoint, height in zip(waypoints, heights):
        center_lat, center_lon = waypoint
        height_meters = height * 0.3048
        
        scale_factor = reference_height_meters / height_meters
        half_sq_side_x_meters = reference_half_sq_side_x * scale_factor
        half_sq_side_y_meters = reference_half_sq_side_y * scale_factor

        utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
        center_utm_x, center_utm_y = gps_to_utm(center_lon, center_lat)

        square_offsets = [
            (-half_sq_side_x_meters, half_sq_side_y_meters),   
            (half_sq_side_x_meters, half_sq_side_y_meters),    
            (half_sq_side_x_meters, -half_sq_side_y_meters),   
            (-half_sq_side_x_meters, -half_sq_side_y_meters),  
            (-half_sq_side_x_meters, half_sq_side_y_meters) 
        ]

        for offset_x, offset_y in square_offsets:
            utm_x = center_utm_x + offset_x
            utm_y = center_utm_y + offset_y
            lat, lon = utm_to_gps(utm_proj, utm_x, utm_y)
            
            waypoint = {
                'latitude': lat,
                'longitude': lon,
                'utm_x': utm_x,
                'utm_y': utm_y,
                'altitude_feet': height,
                'altitude_meters': height_meters
            }
            all_paths.append(waypoint)

        center_waypoint = {
            'latitude': center_lat,
            'longitude': center_lon,
            'utm_x': center_utm_x,
            'utm_y': center_utm_y,
            'altitude_feet': height,
            'altitude_meters': height_meters
        }
        all_paths.append(center_waypoint)

        center_point = {
            'latitude': center_lat,
            'longitude': center_lon,
            'altitude_feet': height,
            'altitude_meters': height_meters,
            'coverage_area_sq_meters': 4 * half_sq_side_x_meters * half_sq_side_y_meters,
            'fov_horizontal_degrees': fov_horizontal,
            'fov_vertical_degrees': fov_vertical
        }
        all_centers.append(center_point)

    return {
        'center_points': all_centers,
        'lap_waypoints': all_paths
    }

def plot_paths(coverage_data):
    plt.figure(figsize=(12, 12))

    for center in coverage_data['center_points']:
        center_utm_x, center_utm_y = gps_to_utm(center['longitude'], center['latitude'])
        plt.plot(center_utm_x, center_utm_y, 'ko', markersize=10)

    waypoints_by_height = {}
    for wp in coverage_data['lap_waypoints']:
        height = wp['altitude_feet']
        if height not in waypoints_by_height:
            waypoints_by_height[height] = {'x': [], 'y': []}
        waypoints_by_height[height]['x'].append(wp['utm_x'])
        waypoints_by_height[height]['y'].append(wp['utm_y'])

    for height, coords in sorted(waypoints_by_height.items()):
        plt.plot(coords['x'], coords['y'], 'o-', label=f'Path at {height} ft', alpha=0.7)

    plt.title('Waypoint Paths')
    plt.xlabel('UTM X (meters)')
    plt.ylabel('UTM Y (meters)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def main():
    waypoints = [
        (-35.3614651, 149.1652373),  
        (-35.3624651, 149.1662373),  
        (-35.3634651, 149.1672373)   
    ]
    
    heights = [10, 15, 20]  

    coverage_data = generate_sequential_paths(
        waypoints=waypoints,
        heights=heights,
        fov_horizontal=90,
        fov_vertical=65
    )

    print(f"Number of waypoints generated: {len(coverage_data['lap_waypoints'])}")
    plot_paths(coverage_data)
    
    package_share_dir = get_package_share_directory("Coverage_Planner")
    waypoints_file = os.path.join(package_share_dir, 'Waypoints', 'science_mission_waypoints.json')
    os.makedirs(os.path.dirname(waypoints_file), exist_ok=True)

    with open(waypoints_file, 'w') as f:
        json.dump(coverage_data, f, indent=4)

if __name__ == "__main__":
    main()