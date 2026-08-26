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
    CRITICAL_DISTANCE = 2.0
    SLOWDOWN_ZONE = 5.0
    def __init__(self):
        self.connection = Connection("udp:0.0.0.0:14550")
        self.master = self.connection.master
        self.other_drones = {}
        self.other_lock = threading.Lock()
        self.arrived_drones = {}
        self.arrived_lock = threading.Lock()
        self.telemetry = Telemetry(self.connection, "drone3", "192.168.199.255", 5005, '0.0.0.0')
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
   # def handle_peer_message(self, msg, addr):
       # pos = msg.get("position",{})
      #  if pos.get("lat") is None or pos.get("lon") is None:
        #  return
       # with self.other_lock:
           # self.other_drones[msg["drone_id"]] = {
            #    "lat": msg["position"]["lat"],
           #     "lon": msg["position"]["lon"],
          #      "alt": msg["position"]["alt"],
         #       "timestamp": msg["timestamp"],
        #    }
    def get_nearby_drones(self, max_age=2.0):
        with self.other_lock:
            now = time.time()
            return {
                k: v for k, v in self.other_drones.items()
                if now - v["timestamp"] < max_age
            }
    def check_separation(self, my_lat, my_lon, target_lat, target_lon):
        my_dist_to_target = self.calc_distance(my_lat, my_lon, target_lat, target_lon)
        for drone_id, pos in self.get_nearby_drones().items():
            if pos["lat"] is None or pos["lon"] is None:
                continue
            dlat = (my_lat - pos["lat"]) * 111320
            dlon =  (my_lon - pos["lon"]) * 111320 * math.cos(math.radians(my_lat))
            dist = math.hypot(dlon, dlat)
            print(f"[SEPARATION CHECK] distance to {drone_id}: {dist:.2f} m")
            if dist < Drone.SLOWDOWN_ZONE:
                logging.info(f"SLOWDOWN_ZONE | near={drone_id} | dist={dist:.2f}m | "
                f"my_pos=({my_lat:.7f},{my_lon:.7f})")
            if dist < Drone.CRITICAL_DISTANCE:
                peer_dist_to_target = self.calc_distance(pos["lat"], pos["lon"], target_lat, target_lon)
                am_farther = ((my_dist_to_target > peer_dist_to_target) or
                              (my_dist_to_target == peer_dist_to_target and self.telemetry.drone_id > drone_id))
                if not am_farther:
                    print(f"[PRIORITY] {drone_id} | dist={dist:.2f} m, but I'm closer to target -- holding course")
                    continue
                print(f"[Emergency]{drone_id} is {dist:.2f} m away")
                logging.warning(f"BREAK | near={drone_id} | dist={dist:.2f}m | "
                                f"my_pos={my_lat:.7f}, {my_lon:.7f} | peer_pos({pos['lat']:.7f},{pos['lon']:.7f})")
                self.set_mode("BRAKE")
                return False
        return True
    def compute_landing_offset(self, index, total, spacing=3.0):
       """Evenly place drones on a small circle around the shared target."""
       if total <= 1:
          return 0.0, 0.0
       radius = spacing / (2 * math.sin(math.pi / total))
       angle = 2 * math.pi * index / total
       d_north = radius * math.cos(angle)
       d_east = radius * math.sin(angle)
       return d_north, d_east

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
    SHARED_LAT, SHARED_LON = 28.315913, 77.359462
    
    # Fly toward the shared area (not landing here -- just converging)
    start_mission.wait_for_goto_position(SHARED_LAT, SHARED_LON, 20)
    print("Reached shared target. Broadcasting arrival.")
    arrival_msg = {
    "type": "ARRIVED",
    "drone_id": start_mission.telemetry.drone_id,
    "timestamp": time.time()
    }
    start_mission.telemetry.sock.sendto(
    json.dumps(arrival_msg).encode(),
    (start_mission.telemetry.broadcast_ip, start_mission.telemetry.port)
    )

    ARRIVAL_GATHER_WINDOW = 6  # seconds -- give slower drones time to also arrive & broadcast
    print(f"Waiting {ARRIVAL_GATHER_WINDOW}s for other drones to check in...")
    time.sleep(ARRIVAL_GATHER_WINDOW)
    # Now figure out final individual slot based on how many drones are here
    with start_mission.other_lock:
        nearby_ids = sorted(set([start_mission.telemetry.drone_id] + list(start_mission.other_drones.keys())))
    my_index = nearby_ids.index(start_mission.telemetry.drone_id)
    total = len(nearby_ids)

    d_north, d_east = start_mission.compute_landing_offset(my_index, total, spacing=5.0)
    slot_lat, slot_lon = start_mission.offset_latlon(SHARED_LAT, SHARED_LON, d_north, d_east)

    print(f"Flying to individual slot {my_index}/{total}: offset north={d_north:.1f} east={d_east:.1f}")
    start_mission.wait_for_goto_position(slot_lat, slot_lon, 20)   # separated by ~3m minimum, not 0m

    start_mission.set_mode("LAND")
    time.sleep(100)
if __name__ == '__main__':
    start()
