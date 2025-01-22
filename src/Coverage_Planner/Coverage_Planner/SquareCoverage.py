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
    
        fov_x_deg (float): Horizontal field of view in degrees
        fov_y_deg (float): Vertical field of view in degrees
        altitude_feet (float): Flying altitude in feet
        altitude_meters (float): Flying altitude converted to meters
        fov_x_meters (float): Horizontal field of view converted to meters at specified altitude
        fov_y_meters (float): Vertical field of view converted to meters at specified altitude
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

def haversine_disance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  
    return c * r

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

def generate_square_coverage_waypoints(lat, lon, square_side, camera, combine_factor):
    """
    Generates a grid of waypoints to cover a square area using a lawnmower pattern.

    """
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    utm_x, utm_y = gps_to_utm(lon, lat)

    step_x = camera.fov_x_meters * combine_factor
    step_y = camera.fov_y_meters*0.75

    x_grids = int(np.ceil(square_side / step_x))
    y_grids = int(np.ceil(square_side / step_y))

    waypoints = []

    for y in range(y_grids):
        x_range = range(x_grids) if y % 2 == 0 else range(x_grids - 1, -1, -1)
        
        for x in x_range:
            grid_x = (x - (x_grids - 1) / 2) * step_x
            grid_y = (y - (y_grids - 1) / 2) * step_y
            abs_utm_x = utm_x + grid_x
            abs_utm_y = utm_y + grid_y
            
            grid_lat, grid_lon = utm_to_gps(utm_proj, abs_utm_x, abs_utm_y)
            waypoints.append({
                'latitude': (grid_lat),
                'longitude': (grid_lon),
                'utm_x': abs_utm_x,
                'utm_y': abs_utm_y
            })

        if y < y_grids - 1:
            transition_x = abs_utm_x
            transition_y = utm_y + ((y + 1 - (y_grids - 1) / 2) * step_y)
            transition_lat, transition_lon = utm_to_gps(utm_proj, transition_x, transition_y)
            waypoints.append({
                'latitude': (transition_lat),
                'longitude': (transition_lon),
                'utm_x': transition_x,
                'utm_y': transition_y
            })
    
    calculate_total_distance(waypoints)

    return {
        'center_point': {'latitude': lat, 'longitude': lon},
        'square_parameters': {
            'side_length': square_side,
            'camera_fov_x': camera.fov_x_meters,
            'camera_fov_y': camera.fov_y_meters,
            'angular_fov_x': camera.fov_x_deg,
            'angular_fov_y': camera.fov_y_deg,
            'combine_factor': combine_factor
        },
        'lap_waypoints': waypoints
    }

def calculate_total_distance(waypoints):
        """
        Calculate the total distance of the path in meters
        """
        total_distance = 0
        for i in range(len(waypoints) - 1):
            lat1 = waypoints[i]["latitude"]
            lon1 = waypoints[i]["longitude"]
            lat2 = waypoints[i + 1]["latitude"]
            lon2 = waypoints[i + 1]["longitude"]
            
            distance = haversine_disance(lat1, lon1, lat2, lon2)
            total_distance += distance
        print(f"Total distance covered in Meters = {total_distance}")
        

def plot_square_coverage(coverage_data):
    """
    Plots the waypoint coverage pattern and square boundary.
    
    """
    center_utm_x, center_utm_y = gps_to_utm(
        coverage_data['center_point']['longitude'],
        coverage_data['center_point']['latitude']
    )

    waypoint_utm_x = [wp['utm_x'] for wp in coverage_data['lap_waypoints']]
    waypoint_utm_y = [wp['utm_y'] for wp in coverage_data['lap_waypoints']]

    plt.figure(figsize=(10, 10))

    side = coverage_data['square_parameters']['side_length']
    half_side = side / 2
    square_x = [
        center_utm_x - half_side,
        center_utm_x + half_side,
        center_utm_x + half_side,
        center_utm_x - half_side,
        center_utm_x - half_side
    ]
    square_y = [
        center_utm_y - half_side,
        center_utm_y - half_side,
        center_utm_y + half_side,
        center_utm_y + half_side,
        center_utm_y - half_side
    ]

    plt.plot(square_x, square_y, 'r--', label='Square Boundary')
    plt.plot(center_utm_x, center_utm_y, 'ro', label='Center Point')
    plt.plot(waypoint_utm_x, waypoint_utm_y, 'go', label='Waypoints')
    plt.plot(waypoint_utm_x, waypoint_utm_y, 'g-', alpha=0.5)

    plt.title('Square Coverage Waypoints')
    plt.xlabel('UTM X (meters)')
    plt.ylabel('UTM Y (meters)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

params = {
        'latitude': -35.3614651,
        'longitude': 149.1652373,
        'square_side': 150,
        'camera_params': {
            'fov_x_deg': 90,
            'fov_y_deg': 65,
            'altitude_feet': 20
        },
        'combine_factor': 1.5,
        'output_file': 'coverage_waypoints.json'
    }

def main(latitude=params['latitude'], longitude=params['longitude'], square_side=params['square_side'], 
         camera_params=params['camera_params'], combine_factor=params['combine_factor'], 
         output_file=params['output_file']):
    """
    Main function to generate and plot waypoints for square coverage.
    
        latitude (float): Center latitude in decimal degrees
        longitude (float): Center longitude in decimal degrees
        square_side (float): Length of square side in meters
        camera_params (dict): Camera parameters including FOV and altitude
        combine_factor (float): Overlap factor for camera coverage
        output_file (str): Filename for saving waypoint data
    """
    camera = Camera(**camera_params)

    coverage_data = generate_square_coverage_waypoints(
        latitude, longitude, square_side, camera, combine_factor
    )

    print(f"No. of Waypoints : {len(coverage_data['lap_waypoints'])}")

    plot_square_coverage(coverage_data)

    package_share_dir = get_package_share_directory("Coverage_Planner")
    waypoints_file = os.path.join(package_share_dir, 'Waypoints', 'coverage_waypoints.json')
    os.makedirs(os.path.dirname(waypoints_file), exist_ok=True)

    with open(waypoints_file, 'w') as f:
        json.dump(coverage_data, f, indent=4)

main()