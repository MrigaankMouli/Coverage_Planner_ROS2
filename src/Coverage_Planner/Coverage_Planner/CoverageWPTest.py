import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
import pyproj
import json
import os

def utm_to_gps(utm_proj, utm_x, utm_y):
    gps_proj = pyproj.Proj(proj='latlong', datum='WGS84')
    lon, lat = pyproj.transform(utm_proj, gps_proj, utm_x, utm_y)
    return lat, lon

with open("Test.json", 'r') as f:
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
        cell = Polygon([(x, y), (x + cell_width, y), (x + cell_width, y + cell_height), (x, y + cell_height)])
        intersection = polygon.intersection(cell)
        if not intersection.is_empty and not isinstance(intersection, Point):
            cells.append(cell)
            centroids.append(cell.centroid)
        y += cell_height
    x += cell_width

centroids_within_polygon = [centroid for centroid in centroids if polygon.contains(centroid)]

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
        boustrophedon_path.extend(sorted(row, key=lambda p: p[0], reverse=True))

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

print("Boustrophedon Path:")
for point in boustrophedon_path:
    print(f"({point[0]}, {point[1]})")

print(len(boustrophedon_path))
