from pymavlink import mavutil

mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

print("🔍 Scanning incoming MAVLink traffic... (Ctrl+C to stop)")
while True:
    msg = mav.recv_match(blocking=True)
    if msg:
        print(f"[{msg.get_type()}] {msg}")

