import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer
import math
import json
import os
from ament_index_python.packages import get_package_share_directory

class Camera:
    """
    Class to define camera parameters and altitude settings for aerial photography.
    """
    def __init__(self, fov_x_deg, fov_y_deg, altitude_feet):
        self.fov_x_deg = fov_x_deg
        self.fov_y_deg = fov_y_deg
        self.altitude_feet = altitude_feet
        self.altitude_meters = self.altitude_feet * 0.3048  
        self.fov_x_meters = self.convert_angular_fov_to_linear(self.fov_x_deg)
        self.fov_y_meters = self.convert_angular_fov_to_linear(self.fov_y_deg)

    def convert_angular_fov_to_linear(self, fov_degrees):
        """
        Converts angular field of view to linear distance on the ground.
        """
        fov_radians = math.radians(fov_degrees)
        half_fov_radians = fov_radians / 2
        return 2 * self.altitude_meters * math.tan(half_fov_radians)

def utm_to_gps(utm_proj, utm_x, utm_y):
    """
    Converts UTM coordinates to GPS coordinates (latitude, longitude).
    """
    gps_proj = Proj(proj='latlong', datum='WGS84')
    transformer = Transformer.from_proj(utm_proj, gps_proj, always_xy=True)
    lon, lat = transformer.transform(utm_x, utm_y)
    return lat, lon

def gps_to_utm(lon, lat):
    """
    Converts GPS coordinates (latitude, longitude) to UTM coordinates.
    """
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    transformer = Transformer.from_proj(Proj(proj='latlong', datum='WGS84'), utm_proj, always_xy=True)
    utm_x, utm_y = transformer.transform(lon, lat)
    return utm_x, utm_y

def generate_square_path(center_lat, center_lon, camera, buffer_factor=1.2, start_corner=0):
    """
    Generates a square path around a waypoint in UTM coordinates, starting from a specified corner.
    """
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    center_utm_x, center_utm_y = gps_to_utm(center_lon, center_lat)
    
    square_size = max(camera.fov_x_meters, camera.fov_y_meters) * buffer_factor
    half_size = square_size / 2
    
    square_offsets = [
        (-half_size, half_size),   
        (half_size, half_size),    
        (half_size, -half_size),   
        (-half_size, -half_size),  
    ]
    
    ordered_offsets = square_offsets[start_corner:] + square_offsets[:start_corner]
    ordered_offsets.append(ordered_offsets[0])
    
    waypoints = []
    for offset_x, offset_y in ordered_offsets:
        utm_x = center_utm_x + offset_x
        utm_y = center_utm_y + offset_y
        lat, lon = utm_to_gps(utm_proj, utm_x, utm_y)
        waypoints.append({
            'latitude': lat,
            'longitude': lon,
            'utm_x': utm_x,
            'utm_y': utm_y,
            'altitude_feet': camera.altitude_feet,
            'altitude_meters': camera.altitude_meters
        })
    
    return waypoints

def generate_sequential_paths(waypoints, heights_per_waypoint, camera_params):
    """
    Generates sequential paths connecting all waypoints in order.
    """
    all_paths = []
    all_centers = []
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    
    for i, (waypoint, height) in enumerate(zip(waypoints, heights_per_waypoint)):
        center_lat, center_lon = waypoint
        center_point = {
            'latitude': center_lat,
            'longitude': center_lon,
            'altitude_feet': height,
            'altitude_meters': height * 0.3048
        }
        all_centers.append(center_point)
        
        if i < 2:  
            camera = Camera(
                fov_x_deg=camera_params['fov_x_deg'],
                fov_y_deg=camera_params['fov_y_deg'],
                altitude_feet=height
            )
            
            if i == 0:
                path_waypoints = generate_square_path(center_lat, center_lon, camera, start_corner=0)
            else:
                prev_point = all_paths[-1]
                curr_utm_x, curr_utm_y = gps_to_utm(center_lon, center_lat)
                corners_utm = []
                for j in range(4):
                    corner_points = generate_square_path(center_lat, center_lon, camera, start_corner=j)
                    first_point = corner_points[0]
                    corners_utm.append((first_point['utm_x'], first_point['utm_y']))
                

                distances = [math.sqrt((prev_point['utm_x'] - x)**2 + (prev_point['utm_y'] - y)**2) 
                           for x, y in corners_utm]
                best_corner = distances.index(min(distances))
                path_waypoints = generate_square_path(center_lat, center_lon, camera, start_corner=best_corner)
            
            if len(all_paths) > 0:
                prev_point = all_paths[-1].copy()
                prev_point['altitude_feet'] = height
                prev_point['altitude_meters'] = height * 0.3048
                all_paths.append(prev_point)
            
            all_paths.extend(path_waypoints)
        else:  
            if len(all_paths) > 0:
                prev_point = all_paths[-1].copy()
                prev_point['altitude_feet'] = height
                prev_point['altitude_meters'] = height * 0.3048
                all_paths.append(prev_point)
            
            utm_x, utm_y = gps_to_utm(center_lon, center_lat)
            all_paths.append({
                'latitude': center_lat,
                'longitude': center_lon,
                'utm_x': utm_x,
                'utm_y': utm_y,
                'altitude_feet': height,
                'altitude_meters': height * 0.3048
            })
    
    coverage_data = {
        'center_points': all_centers,
        'square_parameters': {
            'camera_fov_x_deg': camera_params['fov_x_deg'],
            'camera_fov_y_deg': camera_params['fov_y_deg'],
            'heights': heights_per_waypoint
        },
        'lap_waypoints': all_paths
    }
    
    return coverage_data

def plot_paths(coverage_data):
    """
    Plots the waypoint paths using UTM coordinates
    """
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
    
    for i, (height, coords) in enumerate(sorted(waypoints_by_height.items())):
        plt.plot(coords['x'], coords['y'], f'ro-', 
                label=f'Path at {height}ft', alpha=0.7)

    plt.title('Waypoint Paths at Different Heights')
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
    
    camera_params = {
        'fov_x_deg': 90,
        'fov_y_deg': 65
    }
    
    heights_per_waypoint = [10, 15, 20]  
    
    coverage_data = generate_sequential_paths(
        waypoints=waypoints,
        heights_per_waypoint=heights_per_waypoint,
        camera_params=camera_params
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