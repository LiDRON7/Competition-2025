from pymavlink import mavutil
import time

class Drone:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600,source_system=255,dialect="ardupilotmega", mavlink1=True)  # Adjust the IP and port as necessary
        print("Waiting for heartbeat...")
        print("Manually waiting for HEARTBEAT...")
        while True:
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
            if msg:
                self.mav.target_system = msg.get_srcSystem()
                self.mav.target_component = msg.get_srcComponent()
                print(f"✅ Got HEARTBEAT from system {self.mav.target_system}, component {self.mav.target_component}")
                break

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

    def get_position(self):
        print("📡 Waiting for GLOBAL_POSITION_INT...")
        start = time.time()
        while time.time() - start < 30:  # wait up to 10 seconds
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                print(f"📍 Got position: lat={lat}, lon={lon}")
                return lat, lon
            time.sleep(0.1)
        raise TimeoutError("❌ Timed out waiting for GLOBAL_POSITION_INT")

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


    def takeoff(self, altitude=10):
        # Set mode to GUIDED
        self.set_mode("GUIDED")

        # Arm the drone
        self.arm()
        print("Drone armed, waiting 3 seconds...")
        time.sleep(3)

        # Send takeoff command
        print(f"Taking off to {altitude} meters...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,       # Confirmation
            0, 0, 0, 0,  # Empty params
            0, 0,        # lat, lon (ignored if 0)
            altitude     # Altitude
        )

        time.sleep(10)  # Give time to reach altitude

    def loitering(self, altitude=10, seconds = 10):
        # Loiter for 10 seconds
        print("Loitering for 10 seconds...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
            0,  # Confirmation
            seconds, # Loiter time (in seconds)
            0, 0, 0,
            0, 0,  # lat/lon (ignored if 0)
            altitude
        )

    def set_mode(self, mode):
        mode_id = self.mav.mode_mapping().get(mode)
        if mode_id is None:
            raise Exception(f"Unknown mode: {mode}")
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mode set to {mode}")

    def land(self):
        # Land
        print("Initiating landing...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0,  # lat/lon
            0      # alt
        )
        time.sleep(10)

    def goto_position(self, lat, lon, alt):
        self.set_mode("GUIDED")
        # Convert lat/lon to integers in 1E7 scale (as required by MAVLink)
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        self.mav.mav.set_position_target_global_int_send(
            int(time.time() * 1e6),     # time_boot_ms (microseconds)
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,         # type_mask: enable only position
            lat_int, lon_int, alt,
            0, 0, 0,                    # velocity
            0, 0, 0,                    # acceleration
            0, 0                        # yaw, yaw_rate
        )
        print(f"🛰️ Setpoint sent to: lat={lat}, lon={lon}, alt={alt}")


