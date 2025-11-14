import serial, time
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter

ser = serial.Serial('COM5', 115200, timeout=1)  # ensure COM port is correct and free
ser.flushInput()

with open('speeds.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 15:
        line = ser.readline().decode(errors='ignore')
        if line:
            f.write(line)

ser.close()

# Load CSV file; convert columns to numeric
df = pd.read_csv('speeds.txt', names=['left_speed', 'right_speed', 'time_us'])
df['time_us'] = pd.to_numeric(df['time_us'], errors='coerce')
df['left_speed'] = pd.to_numeric(df['left_speed'], errors='coerce')
df['right_speed'] = pd.to_numeric(df['right_speed'], errors='coerce')
df = df.dropna()

# Convert time to seconds
df['time_s'] = df['time_us'] / 1000000.0

# Plot actual vs target speeds

plt.plot(df['time_s'], df['left_speed'], label='Left Speed', color='blue')
plt.plot(df['time_s'], df['right_speed'], label='Right Speed', color='red')

plt.title(f'Motor Control Performance Test')
plt.ylabel('Speed (cm/s)')
plt.legend()
plt.grid(True)
plt.savefig("speed_test.png", dpi=600);
plt.show()