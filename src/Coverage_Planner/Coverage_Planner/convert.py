import json
import csv

def read_waypoints_from_json(json_file):
    """
    Read waypoints from a JSON file.
    
    Args:
        json_file: Path to JSON file containing waypoints
        
    Returns:
        List of tuples containing (latitude, longitude)
    """
    with open(json_file, 'r') as f:
        data = json.load(f)
        waypoints = [(point['latitude'], point['longitude']) for point in data['lap_waypoints']]
    return waypoints

def create_waypoints_file(waypoints, output_file, start_altitude=113.78, waypoint_altitude=30.0):
    """
    Convert waypoints to QGC format and save as .waypoints file
    
    Args:
        waypoints: List of tuples containing (latitude, longitude)
        output_file: Output filename (will append .waypoints if not present)
        start_altitude: Altitude for the first waypoint in meters
        waypoint_altitude: Altitude for subsequent waypoints in meters
    """
    if not output_file.endswith('.waypoints'):
        output_file += '.waypoints'
    
    header = 'QGC WPL 110\n'
    
    with open(output_file, 'w') as f:
        f.write(header)
        
        for i, (lat, lon) in enumerate(waypoints):
            current = 0 if i == 0 else 0
            frame = 0 if i == 0 else 3  
            alt = start_altitude if i == 0 else waypoint_altitude
            
            # Format: INDEX CURRENT FRAME COMMAND P1 P2 P3 P4 LAT LON ALT AUTOCONTINUE
            line = f"{i}\t{current}\t{frame}\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1\n"
            f.write(line)

if __name__ == "__main__":
    waypoints = read_waypoints_from_json("/home/zero/Desktop/ISDC/cov_ws/src/Coverage_Planner/Waypoints/coverage_waypoints.json")
    
    create_waypoints_file(waypoints, "mission_waypoints")
    print("Waypoints file created successfully!")