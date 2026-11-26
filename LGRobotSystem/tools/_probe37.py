import time
import serial

ser = serial.Serial("COM37", 921600, timeout=0.1)
ser.reset_input_buffer()
b = bytearray()
t0 = time.time()
while time.time() - t0 < 1.5:
    b.extend(ser.read(ser.in_waiting or 1))
ser.close()
print("bytes", len(b))
print("json_open", b.count(b"[LOG] JSON"), "brace", b.count(b"{"),
      "m202.pos", b.count(b"m202.pos"), "sof5a", b.count(b"\x5a"))
i = b.find(b"[LOG] JSON")
if i >= 0:
    print("json sample:", bytes(b[i:i + 300]))
else:
    print("head:", bytes(b[:120]))
    print("tail:", bytes(b[-120:]))
