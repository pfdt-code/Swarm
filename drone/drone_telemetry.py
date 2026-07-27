import copy
import threading
import math
import socket, json, time
class Telemetry:
    def __init__(self, connection, drone_id, broadcast_ip, port,bind_ip = "0.0.0.0"):
        self.connection = connection
        self.master = connection.master
        self.running = False
        self.lock = threading.Lock()
        self.drone_id = drone_id
        self.broadcast_ip = broadcast_ip
        self.port = port
        self.bind_ip = bind_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        self.telemetry_data = {
            "heartbeat":{
                "mode":None,
                "armed":None,
                "system_status":None,
            },
            "position":{
                "lat": None,
                "lon": None,
                "alt": None,

                "heading": None,
                "vx": None,
                "vy": None,
                "vz": None,
            },
            "gps": {
                "fix_type": None,
                "satellites": None,
            },

            "attitude": {
                "roll": None,
                "pitch": None,
                "yaw": None,
            },

            "battery": {
                "voltage": None,
                "current": None,
                "remaining": None,
            },

            "ekf": {
                "flags": None,
            }
        }

    def telemetry_thread(self):
        while self.running:
            with self.connection.master_lock:
                msg = self.master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                self.telemetry_data["heartbeat"]["mode"] = self.master.flightmode
                self.telemetry_data["heartbeat"]["armed"] = self.master.motors_armed()

            elif t == "GLOBAL_POSITION_INT":
                self.telemetry_data["position"]["lat"]=msg.lat / 1e7
                self.telemetry_data["position"]["lon"]=msg.lon / 1e7
                self.telemetry_data["position"]["alt"]=msg.relative_alt / 1000
                self.telemetry_data["position"]["heading"] = msg.hdg / 100
                self.telemetry_data["position"]["vx"]=msg.vx / 100.0
                self.telemetry_data["position"]["vy"]=msg.vy / 100.0
                self.telemetry_data["position"]["vz"]=msg.vz / 100.0

            elif t == "GPS_RAW_INT":
                self.telemetry_data["gps"]["fix_type"]=msg.fix_type
                self.telemetry_data["gps"]["satellites"]= msg.satellites_visible
            elif t == "ATTITUDE":
                self.telemetry_data["attitude"]["roll"]=math.degrees(msg.roll)
                self.telemetry_data["attitude"]["pitch"]=math.degrees(msg.pitch)
                self.telemetry_data["attitude"]["yaw"]=math.degrees(msg.yaw)

            elif t == "BATTERY":
                self.telemetry_data["battery"]["voltage"]=msg.voltage_battery / 100.0
                self.telemetry_data["battery"]["current"]=msg.current_battery / 100.0
                self.telemetry_data["battery"]["remaining"] = msg.remaining_battery

            elif t == "EKF_STATUS_REPORT":
                self.telemetry_data["ekf"]['flag']=msg.flags

    def get_snapshot(self):
        with self.lock:
            snapshot = copy.deepcopy(self.telemetry_data)
        snapshot['drone_id'] = self.drone_id
        snapshot['timestamp'] = time.time()
        return snapshot

    def broadcast_thread(self, interval=0.5):
        while self.running:
            snapshot = self.get_snapshot()
            try:
                packet = json.dumps(snapshot).encode()
                self.sock.sendto(packet, (self.broadcast_ip, self.port))
            except Exception as e:
                print(f"[broadcast_thread] send error: {e}")
            time.sleep(interval)

    def listen_thread(self, on_message=None):
        listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
        listen_sock.bind((self.bind_ip, self.port))
        while self.running:
            try:
                data, addr = listen_sock.recvfrom(4096)
                msg = json.loads(data.decode())
                if msg.get("drone_id") == self.drone_id:
                    continue
                if on_message:
                    on_message(msg,addr)
            except Exception as e:
                print(f"[listen_thread] send error: {e}")

    def start(self, on_peer_message=None):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.telemetry_thread , daemon=True).start()
        threading.Thread(target=self.broadcast_thread, daemon=True).start()
        threading.Thread(target=self.listen_thread, args=(on_peer_message,),daemon=True).start()
    def stop(self):
        self.running = False