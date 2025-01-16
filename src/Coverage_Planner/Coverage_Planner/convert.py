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
        # Convert JSON data to list of tuples
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
    # Ensure output file has .waypoints extension
    if not output_file.endswith('.waypoints'):
        output_file += '.waypoints'
    
    # Create the header line
    header = 'QGC WPL 110\n'
    
    with open(output_file, 'w') as f:
        # Write header
        f.write(header)
        
        # Write waypoints
        for i, (lat, lon) in enumerate(waypoints):
            # Set parameters based on whether it's the first waypoint
            current = 1 if i == 0 else 0
            frame = 0 if i == 0 else 3  # First point global, others relative
            alt = start_altitude if i == 0 else waypoint_altitude
            
            # Format: INDEX CURRENT FRAME COMMAND P1 P2 P3 P4 LAT LON ALT AUTOCONTINUE
            line = f"{i}\t{current}\t{frame}\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.6f}\t1\n"
            f.write(line)

if __name__ == "__main__":
    # Example JSON file structure:

    
    # # Save example JSON to file (for testing)
    # with open("coverage_waypoints.json", "w") as f:
    #     json.dump(example_json, f, indent=4)
    
    # Read waypoints from JSON file
    waypoints = read_waypoints_from_json("coverage_waypoints.json")
    
    # Create the waypoints file
    create_waypoints_file(waypoints, "mission_waypoints")
    print("Waypoints file created successfully!")