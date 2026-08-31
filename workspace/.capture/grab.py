import sys, time, serial

port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
s = serial.Serial()
s.port = port
s.baudrate = 115200
s.timeout = 0.2
# Do not toggle the lines: on the ESP32-S3 USB-Serial/JTAG that can reset the
# chip or drop it into the bootloader.
s.dtr = False
s.rts = False
s.open()
s.dtr = False
s.rts = False

end = time.time() + secs
with open(out, "wb") as f:
    while time.time() < end:
        data = s.read(4096)
        if data:
            f.write(data)
            f.flush()
s.close()
print("captured %d bytes to %s" % (__import__("os").path.getsize(out), out))
