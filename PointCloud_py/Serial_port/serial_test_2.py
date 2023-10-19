import serial
import threading

ser = serial.Serial('COM9', 115200, timeout=1)

def receive_thread():
    while True:
        data = ser.readline()
        if data:
            print('Received:', data.decode())

def send_thread():
    while True:
        data = input('Input data to send:')
        ser.write(data.encode())

receive_t = threading.Thread(target=receive_thread)
send_t = threading.Thread(target=send_thread)
receive_t.start()
send_t.start()