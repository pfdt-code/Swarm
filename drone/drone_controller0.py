import threading
import json
import logging
from pymavlink import mavutil
import math
import time
from drone.drone_telemetry import Telemetry
# from drone.helper import telemetry
from drone.mavlink_manager import Connection
class Drone:
    CRITICAL_DISTANCE = 5.0
    SLOWDOWN_ZONE = 10.0
    NORMAL_SPEED = 5.0
    SLOW_SPEED = 1.0
    CATCT_UP_SPEED = 12.0
    FORMATION_GAP_MAX = 15.0
    KNOWN_SWARM_IDS = ["drone1", "drone2", "drone3"]
    def __init__(self):
        self.connection = Connection("udp:0.0.0.0:14550")
        self.master = self.connection.master
        self.other_drones = {}
        self.other_lock = threading.Lock()
        self.arrived_drones = {}
        self.current_speed = None
        self.arrived_lock = threading.Lock()
        self.telemetry = Telemetry(self.connection, "drone2", "192.168.199.255", 5005, '0.0.0.0')
        self.telemetry.start(on_peer_message=self.handle_peer_message)
        self.lat = None
        self.lon = None
        self.altitude = None
        self.mode = None
        self.armed = False
        self.position_lock = threading.Lock()
        logging.basicConfig(
           filename='drone_events.log',
           level=logging.INFO,
           format='%(asctime)s [%(levelname)s] %(message)s',
           datefmt='%Y-%m-%d %H:%M:%S'
            )
    def decide_landing_slot(self, shared_lat, shared_lon, altitude, spacing=10.0, heading_deg=None):
        known_ids = Drone.KNOWN_SWARM_IDS
        total = len(known_ids)
        my_index = known_ids.index(self.telemetry.drone_id)

        if heading_deg is None:
            cur_lat = self.telemetry.telemetry_data["position"]["lat"]
            cur_lon = self.telemetry.telemetry_data["position"]["lon"]
            heading_deg = self.compute_perpendicular_heading(cur_lat, cur_lon, shared_lat, shared_lon)
            print(f"[AUTO HEADING] Computed perpendicular line heading: {heading_deg:.1f}°")

        print(f"[SLOT DECIDED] Fixed roster: {known_ids} -- I am index {my_index}/{total} (line formation)")
        d_north, d_east = self.compute_line_offset(my_index, total, spacing=spacing, heading_deg=heading_deg)
        target_lat, target_lon = self.offset_latlon(shared_lat, shared_lon, d_north, d_east)

        print(f"Line position {my_index}/{total}: offset north={d_north:.1f} east={d_east:.1f}")
        return target_lat, target_lon
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

    def set_speed(self, speed_m_s):
        if self.current_speed == speed_m_s:
           return

        print(f"[SPEED] Changing speed to {speed_m_s} m/s")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            1,
            speed_m_s,
            -1,
            0, 0, 0, 0
        )

        self.current_speed = speed_m_s
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

    def wait_for_goto_position(self,target_lat, target_lon, altitude, timeout=1200):
        start_time = time.time()
        was_braking = False
        while time.time() - start_time < timeout:


            cur_lat = self.telemetry.telemetry_data["position"]["lat"]
            cur_lon = self.telemetry.telemetry_data["position"]["lon"]
            cur_alt = self.telemetry.telemetry_data["position"]["alt"]
            if not self.check_separation(cur_lat, cur_lon, target_lat, target_lon):
                was_braking = True
                time.sleep(0.5)
                continue
            if was_braking:
                logging.info(f"RESUME | switching back to GUIDED | pos=({cur_lat:.7f},{cur_lon:.7f})")
                self.set_mode("GUIDED")
                was_braking = False

            dist = self.calc_distance(cur_lat, cur_lon, target_lat, target_lon)
            print(f"POS → LAT {cur_lat:.7f} | LON {cur_lon:.7f} | ALT {cur_alt:.2f} | DIST {dist:.1f} m")

            self.goto_position(target_lat, target_lon, altitude)  # keep sending the TARGET, not current pos

            if dist < 2.0 and abs(altitude - cur_alt) < 1.0:
                print("Destination reached")
                return True

            time.sleep(0.5)

        print("Timeout reaching destination")
        return False

    def wait_for_altitude(self, target_altitude, tolerance=1.0, timeout=60):
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
    def handle_peer_message(self, msg, addr):
        if msg.get("type") == "ARRIVED":
            with self.arrived_lock:
                self.arrived_drones[msg["drone_id"]] = msg["timestamp"]
            return

        pos = msg.get("position", {})
        if pos.get("lat") is None or pos.get("lon") is None:
            return
        with self.other_lock:
            self.other_drones[msg["drone_id"]] = {
                "lat": msg["position"]["lat"],
                "lon": msg["position"]["lon"],
                "alt": msg["position"]["alt"],
                "timestamp": msg["timestamp"],
            }
    def get_nearby_drones(self, max_age=2.0):
        with self.other_lock:
            now = time.time()
            return {
                k: v for k, v in self.other_drones.items()
                if now - v["timestamp"] < max_age
            }
    def check_separation(self, my_lat, my_lon, target_lat, target_lon):

        my_dist_to_target = self.calc_distance(
            my_lat,
            my_lon,
            target_lat,
            target_lon
        )

        should_slow = False
        should_brake = False
        should_fast =  False
        for drone_id, pos in self.get_nearby_drones().items():

            if pos["lat"] is None or pos["lon"] is None:
                continue

            # Distance between drones
            dlat = (my_lat - pos["lat"]) * 111320

            dlon = (
                (my_lon - pos["lon"])
                * 111320
                * math.cos(math.radians(my_lat))
            )

            dist = math.hypot(dlon, dlat)

            print(
                f"[SEPARATION] {drone_id}: "
                f"{dist:.2f} m"
            )
            if dist > Drone.FORMATION_GAP_MAX:
                peer_dist_to_target = self.calc_distance(pos["lat"], pos["lon"], target_lat, target_lon)
                if my_dist_to_target > peer_dist_to_target:
	            # I am farther from target AND far from this neighbor -- I need to catch up
                    should_fast = True
                    print(f"[CATCH UP] {drone_id} is {dist:.1f}m away and I'm farther from target -- speeding up")
           
            # ------------------------------------------------
            # SLOWDOWN ZONE
            # ------------------------------------------------
            if dist < Drone.SLOWDOWN_ZONE:

                peer_dist_to_target = self.calc_distance(
                    pos["lat"],
                    pos["lon"],
                    target_lat,
                    target_lon
                )

                print(
                    f"[PRIORITY] "
                    f"Me={my_dist_to_target:.2f}m "
                    f"| {drone_id}={peer_dist_to_target:.2f}m"
                )

                # I am farther from target
                if my_dist_to_target > peer_dist_to_target:

                    should_slow = True

                    logging.info(
                        f"SLOWDOWN | near={drone_id} | "
                        f"distance={dist:.2f}m | "
                        f"my_target_dist={my_dist_to_target:.2f}m | "
                        f"peer_target_dist={peer_dist_to_target:.2f}m"
                    )

                else:

                    print(
                        f"[PRIORITY] {drone_id} is farther "
                        f"from target. I keep normal speed."
                    )

            # ------------------------------------------------
            # CRITICAL DISTANCE
            # ------------------------------------------------

            if dist < Drone.CRITICAL_DISTANCE:

                peer_dist_to_target = self.calc_distance(
                    pos["lat"],
                    pos["lon"],
                    target_lat,
                    target_lon
                )

                # I am farther from target
                if my_dist_to_target > peer_dist_to_target:

                    should_brake = True

                    logging.warning(
                        f"BRAKE | near={drone_id} | "
                        f"dist={dist:.2f}m | "
                        f"my_target={my_dist_to_target:.2f}m | "
                        f"peer_target={peer_dist_to_target:.2f}m"
                    )

                else:

                    print(
                        f"[PRIORITY] {drone_id} is farther "
                        f"from target. I have priority."
                    )

        # ------------------------------------------------
        # APPLY SPEED ONLY ONCE
        # ------------------------------------------------

        if should_brake:

            print("[BRAKE] I am the lower-priority drone")
            self.set_mode("BRAKE")
            return False

        elif should_slow:

            print("[SPEED] Lower priority → 3 m/s")
            self.set_speed(3)
        elif should_fast:
             self.set_speed(12)
             print("SPEED CHANGED TO 12 m/s")
        else:

            print("[SPEED] Normal → 10 m/s")
            self.set_speed(5)

        return True
#    def compute_landing_offset(self, index, total, spacing=3.0):
 #      """Evenly place drones on a small circle around the shared target."""
  #     if total <= 1:
   #       return 0.0, 0.0
    #   radius = spacing / (2 * math.sin(math.pi / total))
     #  angle = 2 * math.pi * index / total
      # d_north = radius * math.cos(angle)
      # d_east = radius * math.sin(angle)
      # return d_north, d_east
    def compute_line_offset(self, index, total, spacing=10.0, heading_deg=0.0):
        """
        Places 'total' drones in a straight line, 'spacing' meters apart,
        centered on the shared point. heading_deg controls the line's
        orientation (0 = north-south line, 90 = east-west line, etc.)
        """
        # Center the formation: index positions run symmetrically around 0
        center_offset = (total - 1) / 2.0
        distance_along_line = (index - center_offset) * spacing

        heading_rad = math.radians(heading_deg)
        d_north = distance_along_line * math.cos(heading_rad)
        d_east = distance_along_line * math.sin(heading_rad)

        return d_north, d_east
    def compute_perpendicular_heading(self, my_lat, my_lon, shared_lat, shared_lon):
        """Returns a heading (degrees) perpendicular to the direction from
           current position toward the shared point -- gives side-by-side formation
           regardless of approach direction."""
        dlat = (shared_lat - my_lat) * 111320
        dlon = (shared_lon - my_lon) * 111320 * math.cos(math.radians(my_lat))
        approach_heading = math.degrees(math.atan2(dlon, dlat))  # heading toward shared point
        perpendicular_heading = (approach_heading + 90) % 360
        return perpendicular_heading
    def offset_latlon(self, lat, lon, d_north, d_east):
       new_lat = lat + (d_north / 111320)
       new_lon = lon + (d_east / (111320 * math.cos(math.radians(lat))))
       return new_lat, new_lon
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
    SHARED_LAT, SHARED_LON = 28.352761, 77.391749

    # Fixed-roster line formation -- no broadcast wait needed, no race condition
    target_lat, target_lon = start_mission.decide_landing_slot(
        SHARED_LAT, SHARED_LON, 20, spacing=10.0, heading_deg=0.0
    )

    print(f"Flying directly to decided slot: ({target_lat:.7f}, {target_lon:.7f})")
    start_mission.wait_for_goto_position(target_lat, target_lon, 20)

    # Final live safety check right before committing to land -- not a fixed wait,
    # just confirms nobody is currently too close
    cur_lat = start_mission.telemetry.telemetry_data["position"]["lat"]
    cur_lon = start_mission.telemetry.telemetry_data["position"]["lon"]
    max_wait = 10
    waited = 0
    while not start_mission.check_separation(cur_lat, cur_lon, target_lat, target_lon) and waited < max_wait:
        print("Not safe to land yet, holding...")
        time.sleep(1)
        waited += 1
        cur_lat = start_mission.telemetry.telemetry_data["position"]["lat"]
        cur_lon = start_mission.telemetry.telemetry_data["position"]["lon"]

    start_mission.set_mode("LAND")
    time.sleep(100)
if __name__ == '__main__':
    start()
