import threading
from pymavlink import mavutil
import math
import time
from drone.drone_telemetry import Telemetry
# from drone.helper import telemetry
from drone.mavlink_manager import Connection
class Drone:
    def __init__(self):
        self.connection = Connection("udp:127.0.0.1:14550")
        self.master = self.connection.master
        self.telemetry = Telemetry(self.connection, "drone1", '0.0.0.0',5005)
        self.telemetry.start()
        self.lat = None
        self.lon = None
        self.altitude = None
        self.mode = None
        self.armed = False
        self.other_drones = {}
        self.position_lock = threading.Lock()
        self.other_lock = threading.Lock()

    def is_ready_to_arm(self):

        gps = self.telemetry.telemetry_data["gps"]
        battery = self.telemetry.telemetry_data["battery"]
        ekf = self.telemetry.telemetry_data["ekf"]

        if gps["fix_type"] is None:
            return False, "Waiting for GPS"

        if gps["fix_type"] < 3:
            return False, "GPS not fixed"

        if battery["remaining"] is not None and battery["remaining"] < 20:
            return False, "Battery low"

        return True, "Ready to arm"


    def arm_drone(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
    def gps_lock_wait(self):
        while True:
            gps = self.telemetry.telemetry_data["gps"]
            fix_type = gps["fix_type"]
            if fix_type is not None and fix_type >= 3:
                print(f"GPS Fixed {fix_type}")
                return
            time.sleep(1)
    def set_mode(self,mode):
        mode_id = self.master.mode_mapping()[mode]
        with self.connection.master_lock:
            self.master.set_mode(mode_id)

    def wait_for_mode(self):
        target_mode = "GUIDED"
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.telemetry.telemetry_data["heartbeat"]["mode"]
            if msg == target_mode:
                print(msg)
                return True
        print(f"Timeout: vehicle did not reach mode {target_mode}")
        return False

    def takeoff(self,target_altitude):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, float('nan'), 0, 0, target_altitude
        )

    def goto_position(self,lat, lon, altitude):
        self.master.mav.set_position_target_global_int_send(
            time_boot_ms=0,
            target_system=self.master.target_system,
            target_component=self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            type_mask=0b0000111111111000,
            lat_int=int(lat * 1e7),
            lon_int=int(lon * 1e7),
            alt=altitude,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0,
            yaw=0,
            yaw_rate=0
        )

    def calc_distance(self,lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def wait_for_goto_position(self,target_lat, target_lon, altitude, timeout=120):
        start_time = time.time()
        while time.time() - start_time < timeout:


            cur_lat = self.telemetry.telemetry_data["position"]["lat"]
            cur_lon = self.telemetry.telemetry_data["position"]["lon"]
            cur_alt = self.telemetry.telemetry_data["position"]["alt"]

            dist = self.calc_distance(cur_lat, cur_lon, target_lat, target_lon)
            print(f"POS → LAT {cur_lat:.7f} | LON {cur_lon:.7f} | ALT {cur_alt:.2f} | DIST {dist:.1f} m")

            self.goto_position(target_lat, target_lon, altitude)  # keep sending the TARGET, not current pos

            if dist < 2.0 and abs(altitude - cur_alt) < 1.0:
                print("Destination reached")
                return True

            time.sleep(0.5)

        print("Timeout reaching destination")
        return False

    def wait_for_altitude(self, target_altitude, tolerance=1.0, timeout=30):
        start = time.time()
        while time.time() - start < timeout:
            msg = self.telemetry.telemetry_data["position"]["alt"]
            if msg is None:
                continue
            cur_alt = msg
            print(f"Climbing... ALT {cur_alt} m")
            if cur_alt >= target_altitude - tolerance:
                print("Target altitude reached")
                return True
            time.sleep(0.5)
        print("Timeout waiting for altitude")
        return False
def start():
    start_mission = Drone()
    while True:
        ready, reason = start_mission.is_ready_to_arm()

        print(reason)

        if ready:
            break

        time.sleep(1)

    # Now prepare the vehicle
    start_mission.set_mode("GUIDED")
    start_mission.wait_for_mode()

    while True:

        start_mission.arm_drone()

        if start_mission.master.motors_armed():
            print("Armed")
            break

        print("Waiting for vehicle to become armable...")
        time.sleep(5)
    print("Motors armed")
    start_mission.takeoff(20)
    start_mission.wait_for_altitude(20)
    start_mission.wait_for_goto_position(28.309369, 77.354522, 20)

if __name__ == '__main__':
    start()