import math
from dronekit import connect, VehicleMode
import time
import can

bus = can.interface.Bus(channel='can0', bustype='socketcan_native')

connection_string = "udp:127.0.0.1:14550"

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

vehicle.wait_ready('autopilot_version')

can_status = {
    "AUTO": 0,
    "MANUAL": 1,
}

# Get all vehicle attributes (state)
while True:
    lat = format(abs(round(vehicle.location.global_relative_frame.lat*100000)), '024b')
    lon = format(round(vehicle.location.global_relative_frame.lon*100000), '024b')
    yaw = format(round(180 + vehicle.attitude.yaw * 180 / math.pi), '016b')
    try:
        mode = can_status[vehicle.mode.name]
    except KeyError:
        mode = 3
    # 緯度経度角度
    msgF1 = can.Message(
        arbitration_id=0xF1,
        data=[
            int(lat[0:8], 2),
            int(lat[8:16], 2),
            int(lat[16:24], 2),
            int(lon[0:8], 2),
            int(lon[8:16], 2),
            int(lon[16:24], 2),
            int(yaw[0:8], 2), int(yaw[8:16], 2)
        ],
        extended_id=False
    )
    bus.send(msgF1)

    # # # システム状態
    msgF2 = can.Message(arbitration_id=0xF1, data=[mode, int(vehicle.gps_0.fix_type), 255, 255], extended_id=False)
    bus.send(msgF2)

    # # 北大からの司令
    canmsg = bus.recv()
    if canmsg.arbitration_id == 0xF0:
        data = format(canmsg.data[0], '08b')
        if data[1] == '1':
            vehicle.mode = VehicleMode("AUTO")
        else:
            vehicle.mode = VehicleMode("MANUAL")

    time.sleep(1)

vehicle.close()

print("Completed")
