import serial, time

ser = serial.Serial('COM4', 115200, timeout=1)  # ensure COM port is correct and free

with open('speeds.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 15:
        line = ser.readline().decode(errors='ignore')
        if line:
            f.write(line)

ser.close()
