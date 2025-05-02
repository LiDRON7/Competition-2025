from pymavlink import mavutil
import time
import math

# === Square-style GeoFence ===
class GeoFence:
    def __init__(self, center_lat, center_lon, width_m, height_m, max_alt):
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.max_alt = max_alt

        # Convert meters to degrees
        self.lat_deg_per_meter = 1 / 111320
        self.lon_deg_per_meter = 1 / (111320 * math.cos(math.radians(center_lat)))

        # Half-dimensions
        half_height_deg = (height_m / 2) * self.lat_deg_per_meter
        half_width_deg = (width_m / 2) * self.lon_deg_per_meter

        # Rectangle bounds
        self.lat_min = center_lat - half_height_deg
        self.lat_max = center_lat + half_height_deg
        self.lon_min = center_lon - half_width_deg
        self.lon_max = center_lon + half_width_deg

    def is_within_bounds(self, lat, lon, alt):
        return (
            self.lat_min <= lat <= self.lat_max and
            self.lon_min <= lon <= self.lon_max and
            0 <= alt <= self.max_alt
        )


# === Drone Class ===
class Drone:
    def __init__(self):
        self.mav = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255, dialect="ardupilotmega", mavlink1=True)
        print("Waiting for heartbeat...")
        while True:
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
            if msg:
                self.mav.target_system = msg.get_srcSystem()
                self.mav.target_component = msg.get_srcComponent()
                print(f"✅ Got HEARTBEAT from system {self.mav.target_system}, component {self.mav.target_component}")
                break
        self.boot_time = time.time()

        self.geofence = GeoFence(
            center_lat=18.2098,
            center_lon=-67.1395,
            width_m=50,     # 50 meters wide (longitude)
            height_m=80,    # 80 meters tall (latitude)
            max_alt=10
        )


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

    def takeoff(self, altitude=10):
        self.set_mode("GUIDED")
        self.arm()
        print("Drone armed. Taking off...")
        time.sleep(3)
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        time.sleep(10)

    def land(self):
        print("Initiating landing...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(10)

    def return_to_launch(self):
        print("Returning to launch...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0
        )

    def goto_position(self, lat, lon, alt):
        if not self.geofence.is_within_bounds(lat, lon, alt):
            print(f"❌ Position ({lat}, {lon}, {alt}) is outside of square geofence.")
            self.return_to_launch()
            return

        self.set_mode("GUIDED")
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        time_boot_ms = int((time.time() - self.boot_time) * 1000)

        self.mav.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            lat_int, lon_int, alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        print(f"🛰️ Setpoint sent to: lat={lat}, lon={lon}, alt={alt}")
