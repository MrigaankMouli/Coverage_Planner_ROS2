import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import pyproj
import json

def utm_to_gps(utm_proj, utm_x, utm_y):
    gps_proj = pyproj.Proj(proj='latlong', datum='WGS84')
    lon, lat = pyproj.transform(utm_proj, gps_proj, utm_x, utm_y)
    return lat, lon

def generate_coverage_waypoints(json_file_path, cell_width, cell_height, alt):
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)
    latitudes = [point['latitude'] for point in data['waypoints']]
    longitudes = [point['longitude'] for point in data['waypoints']]
    points = list(zip(longitudes, latitudes))

    utm_proj = pyproj.Proj(proj='utm', zone=18, ellps='WGS84')
    utm_points = [utm_proj(lon, lat) for lon, lat in points]
    polygon = Polygon(utm_points)

    min_x, min_y, max_x, max_y = polygon.bounds

    num_cells_x = int(np.ceil((max_x - min_x) / cell_width))
    num_cells_y = int(np.ceil((max_y - min_y) / cell_height))

    centroids = []
    for i in range(num_cells_x):
        for j in range(num_cells_y):
            cell_x = min_x + i * cell_width
            cell_y = min_y + j * cell_height
            cell = Polygon([(cell_x, cell_y), (cell_x + cell_width, cell_y), (cell_x + cell_width, cell_y + cell_height), (cell_x, cell_y + cell_height)])
            if polygon.intersects(cell):
                if polygon.contains(cell):
                    centroids.append(cell.centroid)
                else:
                    clipped_cell = cell.intersection(polygon)
                    centroids.append(clipped_cell.centroid)



    gps_centroids = [utm_to_gps(utm_proj, point.x, point.y) for point in centroids]
    gps_centroids_dict = {"waypoints": [{"latitude": lat, "longitude": lon, "altitude": alt} for lat, lon in gps_centroids]}

    return gps_centroids_dict


json_file_path = ''
alt = 10
gps_centroids_dict = generate_coverage_waypoints(
    json_file_path,
    cell_width=15,
    cell_height=10,
    _alt = alt
)

output_json_file_path = ''
with open(output_json_file_path, 'w') as output_json_file:
    json.dump(gps_centroids_dict, output_json_file, indent=4)