import threading

from pymavlink import mavutil
class Connection:
    def __init__(self, address):
        self.master = mavutil.mavlink_connection(address)
        self.master.wait_heartbeat()
        self.master_lock = threading.Lock()