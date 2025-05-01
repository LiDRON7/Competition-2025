from pymavlink import mavutil

class RC:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud = 57600)  # Adjust the IP and port as necessary
    
    def arm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
            )
        
    def disarm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    def set_speed(self, speed):
        # Speed is in meters per second
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,       # Confirmation
            1,       # Speed type (1 for ground speed)
            speed,   # Speed (m/s)
            -1,      # Throttle (ignored)
            0, 0, 0, 0, 0  # Parameters 4-8 (ignored)
        )

    def get_position(self):
        # Request global position information
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)  # Request position data at 1 Hz

        # Wait for the global position message
        while True:
            msg = self.mav.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
            if msg is not None:
                # Convert latitude and longitude to degrees
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                break
        return lat, lon

    def send_waypoint(self, lat, lon):
        msg = mavutil.mavlink.MAVLink_mission_item_message(
            self.mav.target_system, self.mav.target_component,
            0,                      # Sequence
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,                      # Current
            0,                      # Autocontinue
            0,                      # Param 1 (Hold time in seconds)
            0,                      # Param 2 (Acceptance radius in meters)
            0,                      # Param 3 (Pass through to waypoint)
            0,                      # Param 4 (Yaw angle)
            lat,                    # Latitude
            lon,                    # Longitude
            0)                    # Altitude

        # pos = self.get_position()
        self.mav.mav.send(msg)
        # print("YES")
