 #!/usr/bin/env python3
 # -*- coding: utf-8 -*-
"""
 SITL 验证脚本 — 模拟 Nano 向飞控发送 NAMED_VALUE_FLOAT
 
 用法:
   1. 启动 SITL:  ./Tools/autotest/sim_vehicle.py -v Rover --map --console
   2. 另开终端:   python3 Tools/scripts/test_sitl_usv.py
   3. QGC 连接 TCP 127.0.0.1:5760 → MAVLink Inspector 验证
 """
from pymavlink import mavutil
import time
import sys
 
SITL_ADDR = "tcp:127.0.0.1:5762"
 
 
def main():
     print(f"Connecting to SITL at {SITL_ADDR} ...")
     conn = mavutil.mavlink_connection(SITL_ADDR)
     conn.wait_heartbeat()
     print(f"Connected  sysid={conn.target_system} compid={conn.target_component}")
 
     pkt = 0
     try:
         while True:
             t = int(time.time() * 1000) & 0xFFFFFFFF
             pkt = (pkt + 1) % 65536
 
             conn.mav.named_value_float_send(t, b"USV_VOLT\x00\x00", 12.56)
             conn.mav.named_value_float_send(t, b"USV_ABS\x00\x00\x00", 0.432)
             conn.mav.named_value_float_send(t, b"PUMP_X\x00\x00\x00\x00", 45.0)
             conn.mav.named_value_float_send(t, b"PUMP_Y\x00\x00\x00\x00", 90.0)
             conn.mav.named_value_float_send(t, b"PUMP_Z\x00\x00\x00\x00", 0.0)
             conn.mav.named_value_float_send(t, b"PUMP_A\x00\x00\x00\x00", 30.0)
             conn.mav.named_value_float_send(t, b"USV_STAT\x00\x00", 1.0)
             conn.mav.named_value_float_send(t, b"USV_PKT\x00\x00\x00", float(pkt))
 
             sys.stdout.write(f"\rSent pkt={pkt}  VOLT=12.56  ABS=0.432  STAT=1")
             sys.stdout.flush()
             time.sleep(0.2)  # 5 Hz
     except KeyboardInterrupt:
         print("\nStopped.")
 
 
if __name__ == "__main__":
     main()