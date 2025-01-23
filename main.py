from pymavlink import mavutil
import time
from ezdxf import readfile
from pyproj import Transformer

def dxf_to_latlong(dxf_file, home_lat, home_lon):
    transformer = Transformer.from_crs(
        "epsg:4326",
        "epsg:3857",
        always_xy=True
    )

    home_x, home_y = transformer.transform(home_lon, home_lat)
    doc = readfile(dxf_file)
    msp = doc.modelspace()
    waypoints = []

    for entity in msp:
        if entity.dxftype() in ["LINE", "POINT"]:
            points = [entity.dxf.start, entity.dxf.end] if entity.dxftype() == "LINE" else [entity.dxf.location]
            
            for point in points:
                dx, dy = point[0], point[1]
                global_x = home_x + dx
                global_y = home_y + dy

                lon, lat = transformer.transform(global_x, global_y, direction="INVERSE")
                waypoints.append((lat, lon))

    return waypoints

def setup_mavlink_connection(connection_string):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat(timeout=10)
    print(f"Connected to drone (System {master.target_system})")
    return master

def prepare_drone_mission(master, home_lat, home_lon, waypoints):
    # Set home point
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0, 1, 0, 0, 0, home_lat, home_lon, 0
    )

    # Get mode mapping
    mode_mapping = master.mode_mapping()
    guided_mode = mode_mapping.get('GUIDED')
    
    if guided_mode is not None:
        # Set mode to GUIDED
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            guided_mode
        )
        time.sleep(1)

    # Arm drone
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

    # Takeoff
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, 10.0
    )
    time.sleep(5)

    # Clear existing mission
    master.waypoint_clear_all_send()
    
    # Send mission count
    master.waypoint_count_send(len(waypoints))
    
    # Upload mission items
    for i, (lat, lon) in enumerate(waypoints):
        master.mav.mission_item_send(
            master.target_system, 
            master.target_component, 
            i,
            3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            16,  # MAV_CMD_NAV_WAYPOINT
            0, 1, 0, 0, 0, 0,
            lat, lon, 10.0
        )
    
    # Start mission
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def main():
    connection_string = input('Enter MAVLink connection (e.g., udp:127.0.0.1:14550): ')
    home_lat = float(input('Enter home point latitude: '))
    home_lon = float(input('Enter home point longitude: '))
    dxf_file_path = input('Enter DXF file path: ')

    master = setup_mavlink_connection(connection_string)
    waypoints = dxf_to_latlong(dxf_file_path, home_lat, home_lon)

    prepare_drone_mission(master, home_lat, home_lon, waypoints)
    print(f"Mission initiated with {len(waypoints)} waypoints")

if __name__ == "__main__":
    main()