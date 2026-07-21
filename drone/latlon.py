import time

import math

from drone.helper import telemetry
from drone_telemetry import Telemetry, mavutil

fc_telemetry  = Telemetry()


def arm_drone():
    fc_telemetry.master.mav.command_long_send(
        fc_telemetry.master.target_system,
        fc_telemetry.master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0, 0, 0, 0 , 0 ,0
    )

def set_mode(mode):
    mode_id = fc_telemetry.master.mode_mapping()[mode]
    fc_telemetry.master.set_mode(mode_id)

def wait_for_mode():
    mode_id = fc_telemetry.master.mode_mapping()['GUIDED']
    start_time = time.time()
    while time.time() - start_time < 5:
        mode = fc_telemetry.heartbeat
        if mode is None:
            break
        if mode.custom_mode == mode_id:
            print(f"vehicle is now in mode {mode_id}")
            return True
    print(f"Timeout: vehicle did not reach mode {mode_id}")
    return False

def takeoff(target_altitude):
    fc_telemetry.master.mav.command_long_send(
        fc_telemetry.master.target_system,
        fc_telemetry.master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0, float('nan'), 0, 0, target_altitude
    )
def goto_position(lat, lon, altitude):
    fc_telemetry.master.mav.set_position_target_global_int_send(
        time_boot_ms=0,
        target_system=fc_telemetry.master.target_system,
        target_component=fc_telemetry.master.target_component,
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

def calc_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def wait_for_goto_position(target_lat, target_lon, altitude, timeout=120):
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg is None:
            continue

        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        cur_alt = msg.relative_alt / 1000.0

        dist = calc_distance(cur_lat, cur_lon, target_lat, target_lon)
        print(f"POS → LAT {cur_lat:.7f} | LON {cur_lon:.7f} | ALT {cur_alt:.2f} | DIST {dist:.1f} m")

        goto_position(target_lat, target_lon, altitude)  # keep sending the TARGET, not current pos

        if dist < 2.0 and abs(altitude - cur_alt) < 1.0:
            print("Destination reached")
            return True

        time.sleep(0.5)

    print("Timeout reaching destination")
    return False
def wait_for_altitude(target_altitude, tolerance=1.0, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg is None:
            continue
        cur_alt = msg.relative_alt / 1000.0
        print(f"Climbing... ALT {cur_alt:.2f} m")
        if cur_alt >= target_altitude - tolerance:
            print("Target altitude reached")
            return True
        time.sleep(0.5)
    print("Timeout waiting for altitude")
    return False
set_mode('GUIDED')
wait_for_mode()
arm_drone()
master.motors_armed_wait()
print("Motors armed")
takeoff(20)
wait_for_altitude(20)
wait_for_goto_position(51.5045,0.0867, 20)