from ezdxf import readfile
from pyproj import Transformer

def dxf_to_latlong(dxf_file, origin_lat, origin_lon):
    """
    Convert DXF coordinates to geographic coordinates relative to origin point
    
    Args:
        dxf_file (str): Path to DXF file
        origin_lat (float): Origin latitude
        origin_lon (float): Origin longitude
    
    Returns:
        list: List of (latitude, longitude) tuples
    """
    # Create transformer
    transformer = Transformer.from_crs(
        "epsg:4326",  # WGS84 (latitude/longitude)
        "epsg:3857",  # Web Mercator projection
        always_xy=True
    )

    # Convert origin point to global Cartesian coordinates
    origin_x, origin_y = transformer.transform(origin_lon, origin_lat)

    # Load DXF file
    doc = readfile(dxf_file)
    msp = doc.modelspace()
    
    waypoints = []

    # Iterate through entities
    for entity in msp:
        if entity.dxftype() in ["LINE", "POINT"]:
            # Process points from entities
            points = [entity.dxf.start, entity.dxf.end] if entity.dxftype() == "LINE" else [entity.dxf.location]
            
            for point in points:
                dx, dy = point[0], point[1]
                global_x = origin_x + dx
                global_y = origin_y + dy

                # Convert back to lat/lon
                lon, lat = transformer.transform(global_x, global_y, direction="INVERSE")
                waypoints.append((lat, lon))

    return waypoints

def main():
    # User inputs
    dxf_file_path = input('Enter DXF file path: ')
    origin_lat = float(input('Enter origin latitude: '))
    origin_lon = float(input('Enter origin longitude: '))

    # Convert coordinates
    waypoints = dxf_to_latlong(dxf_file_path, origin_lat, origin_lon)

    # Print results
    print("\nConverted Waypoints:")
    for i, (lat, lon) in enumerate(waypoints, 1):
        print(f"Point {i}: Lat {lat}, Lon {lon}")

if __name__ == "__main__":
    main()