import socket
import json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,  socket.SO_REUSEADDR, 1)
s.bind(('0.0.0.0', 5005))
print("pi 1 listening")
while True:
    data, addr = s.recvfrom(4096)
    msg = json.loads(data.decode())
    print(f"Pi 1 got packet from {addr}")
    print(f"From {msg["drone_id"]} ({addr[0]}):",
    f"lat={msg['position']['lat']} lon={msg['position']['lon']}"
)

