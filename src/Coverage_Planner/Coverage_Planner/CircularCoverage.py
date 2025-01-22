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

def generate_coverage(lat, lon, radius, camera, combine_factor):
    """Generates waypoints to cover a circular area in a Boustrophedon Pattern."""
    from pyproj import Proj
    import numpy as np
    
    utm_proj = Proj(proj="utm", zone=18, ellps="WGS84")
    utm_x, utm_y = gps_to_utm(lon, lat)
    step_x = camera.fov_x_meters * combine_factor
    step_y = camera.fov_y_meters
    waypoints = []
    
    num_rows = int(2 * radius / step_y) + 1
    y_positions = np.linspace(-radius, radius, num_rows)
    
    for i, y_offset in enumerate(y_positions):
        half_chord = np.sqrt(radius**2 - y_offset**2)
        
        num_points = int(2 * half_chord / step_x) + 1
        x_positions = np.linspace(-half_chord, half_chord, num_points)
        
        if len(waypoints) > 0 and i % 2 == 1:
            x_positions = x_positions[::-1]
            
        for x_offset in x_positions:
            abs_utm_x = utm_x + x_offset
            abs_utm_y = utm_y + y_offset
            grid_lat, grid_lon = utm_to_gps(utm_proj, abs_utm_x, abs_utm_y)
            waypoints.append({
                'latitude': grid_lat,
                'longitude': grid_lon,
                'utm_x': abs_utm_x,
                'utm_y': abs_utm_y
            })
    
    return {
        'center_point': {'latitude': lat, 'longitude': lon},
        'circle_parameters': {
            'radius': radius,
            'camera_fov_x': camera.fov_x_meters,
            'camera_fov_y': camera.fov_y_meters,
            'angular_fov_x': camera.fov_x_deg,
            'angular_fov_y': camera.fov_y_deg,
            'combine_factor': combine_factor
        },
        'lap_waypoints': waypoints
    }
def plot_coverage(coverage_data):
    """
    Plots the waypoint coverage pattern and circular boundary.
    """
    center_utm_x, center_utm_y = gps_to_utm(
        coverage_data['center_point']['longitude'],
        coverage_data['center_point']['latitude']
    )
    waypoint_utm_x = [wp['utm_x'] for wp in coverage_data['lap_waypoints']]
    waypoint_utm_y = [wp['utm_y'] for wp in coverage_data['lap_waypoints']]

    plt.figure(figsize=(10, 10))
    
    circle = plt.Circle(
        (center_utm_x, center_utm_y),
        coverage_data['circle_parameters']['radius'],
        fill=False,
        linestyle='--',
        color='r',
        label='Circle Boundary'
    )
    plt.gca().add_patch(circle)
    
    plt.plot(center_utm_x, center_utm_y, 'ro', label='Center Point')
    plt.plot(waypoint_utm_x, waypoint_utm_y, 'go', label='Waypoints')
    plt.plot(waypoint_utm_x, waypoint_utm_y, 'g-', alpha=0.5)
    plt.title('Circular Coverage Waypoints')
    plt.xlabel('UTM X (meters)')
    plt.ylabel('UTM Y (meters)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

params = {
        'latitude': -35.3614651,
        'longitude': 149.1652373,
        'radius': 200,
        'camera_params': {
            'fov_x_deg': 90,
            'fov_y_deg': 65,
            'altitude_feet': 24
        },
        'combine_factor': 5,
        'output_file': 'coverage_waypoints.json'
    }

def main(latitude=params['latitude'], longitude=params['longitude'], radius=params['radius'], 
         camera_params=params['camera_params'], combine_factor=params['combine_factor'], 
         output_file=params['output_file']):
    """
    Main function to generate and plot waypoints for square coverage.
    
        latitude (float): Center latitude in decimal degrees
        longitude (float): Center longitude in decimal degrees
        radius (float): radius of circle in meters
        camera_params (dict): Camera parameters including FOV and altitude
        combine_factor (float): Overlap factor for camera coverage
        output_file (str): Filename for saving waypoint data
    """
    camera = Camera(**camera_params)

    coverage_data = generate_coverage(
        latitude, longitude, radius, camera, combine_factor
    )

    print(f"No. of Waypoints : {len(coverage_data['lap_waypoints'])}")

    plot_coverage(coverage_data)

    package_share_dir = get_package_share_directory("Coverage_Planner")
    waypoints_file = os.path.join(package_share_dir, 'Waypoints', 'coverage_waypoints.json')
    os.makedirs(os.path.dirname(waypoints_file), exist_ok=True)

    with open(waypoints_file, 'w') as f:
        json.dump(coverage_data, f, indent=4)


if __name__ =='__main__':
    main()