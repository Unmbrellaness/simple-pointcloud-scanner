# simple inquiry example
# import bluetooth
import serial


# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# print("Found {} devices.".format(len(nearby_devices)))
#
# for addr, name in nearby_devices:
#     print("  {} - {}".format(addr, name))

ser = serial.Serial('COM9', 115200, timeout=1)
if ser.isOpen():
    print("open success")
else:
    print("open failed")

data = "1 90 60 0.5 0.5 110 120 0 0"
stop = "2"
ser.write(stop.encode("utf-8"))

while True:
    # data = input()
    # ser.write(data.encode("utf-8"))
    line = ser.readline().decode().strip()  # 读取一行数据并解码
    if line:
        print(line)

