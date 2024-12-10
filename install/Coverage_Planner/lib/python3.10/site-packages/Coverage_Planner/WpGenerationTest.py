import numpy as np
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer
import math
import json

def convert_angular_fov_to_linear(fov_degrees, altitude):
    fov_radians = math.radians(fov_degrees)
    half_fov_radians = fov_radians / 2
    linear_fov = 2 * altitude * math.tan(half_fov_radians)
    return linear_fov

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

def generate_square_coverage_waypoints(lat, lon, square_side, camera_fov_x, camera_fov_y, camera_fov_x_deg, camera_fov_y_deg, combine_factor=2):
    utm_proj = Proj(proj='utm', zone=18, ellps='WGS84')
    utm_x, utm_y = gps_to_utm(lon, lat)
    
    half_side = square_side / 2
    x_grids = int(np.ceil(square_side / camera_fov_x))
    y_grids = int(np.ceil(square_side / camera_fov_y))
    
    waypoints = []
    for y in range(0, y_grids, combine_factor):
        x_range = range(0, x_grids, combine_factor) if y % 2 == 0 else range(x_grids-1, -1, -combine_factor)
        for x in x_range:
            grid_x = (x - (x_grids-1)/2) * camera_fov_x
            grid_y = (y - (y_grids-1)/2) * camera_fov_y
            abs_utm_x = utm_x + grid_x
            abs_utm_y = utm_y + grid_y
            grid_lat, grid_lon = utm_to_gps(utm_proj, abs_utm_x, abs_utm_y)
            waypoints.append({
                'latitude': int(grid_lat*1e7),
                'longitude': int(grid_lon*1e7),
                'utm_x': abs_utm_x,
                'utm_y': abs_utm_y
            })
    
    return {
        'center_point': {'latitude': lat, 'longitude': lon},
        'square_parameters': {
            'side_length': square_side,
            'camera_fov_x': camera_fov_x,
            'camera_fov_y': camera_fov_y,
            'angular_fov_x': camera_fov_x_deg,
            'angular_fov_y': camera_fov_y_deg
        },
        'waypoints': waypoints
    }

def plot_square_coverage(coverage_data):
    center_utm_x, center_utm_y = gps_to_utm(
        coverage_data['center_point']['longitude'],
        coverage_data['center_point']['latitude']
    )
    
    waypoint_utm_x = [wp['utm_x'] for wp in coverage_data['waypoints']]
    waypoint_utm_y = [wp['utm_y'] for wp in coverage_data['waypoints']]
    
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

def main():
    latitude = -35.3614651
    longitude = 149.1652373
    square_side = 400
    
    camera_fov_x_deg = 90  
    camera_fov_y_deg = 65  
    alt_feet = 22
    
    altitude = alt_feet * 0.3048
    
    camera_fov_x = convert_angular_fov_to_linear(camera_fov_x_deg, altitude)
    camera_fov_y = convert_angular_fov_to_linear(camera_fov_y_deg, altitude)
    
    coverage_data = generate_square_coverage_waypoints(
        latitude, longitude, square_side, camera_fov_x, camera_fov_y, camera_fov_x_deg, camera_fov_y_deg, combine_factor=2
    )
    
    plot_square_coverage(coverage_data)

    print(f"No. of Waypoints : {len(coverage_data['waypoints'])}")
    
    with open('coverage_waypoints.json', 'w') as f:
        json.dump(coverage_data, f, indent=4)

if __name__ == "__main__":
    main()
