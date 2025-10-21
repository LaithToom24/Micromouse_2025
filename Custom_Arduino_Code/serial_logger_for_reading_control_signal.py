import serial, time
import pandas as pd
import matplotlib.pyplot as plt

ser = serial.Serial('COM4', 115200, timeout=1)  # ensure COM port is correct and free

with open('controls.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 15:
        line = ser.readline().decode(errors='ignore')
        if line:
            f.write(line)

ser.close()

# Load CSV file; convert columns to numeric
df = pd.read_csv('controls.txt', names=['left_speed', 'right_speed', 'time_us'])
df['time_us'] = pd.to_numeric(df['time_us'], errors='coerce')
df['left_speed'] = pd.to_numeric(df['left_speed'], errors='coerce')
df['right_speed'] = pd.to_numeric(df['right_speed'], errors='coerce')
df = df.dropna()

# Convert time to seconds
df['time_s'] = df['time_us'] / 1000000.0

# Plot actual vs target speeds
plt.figure(figsize=(10,5))
plt.plot(df['time_s'], df['left_speed'], label='Left Actual', color='blue')
plt.plot(df['time_s'], df['right_speed'], label='Right Actual', color='red')

plt.xlabel('Time (s)')
plt.ylabel('PWM')
plt.title('Control signals')
plt.legend()
plt.grid(True)
plt.savefig("control_test.png");
plt.show()
