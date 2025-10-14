import serial, time
import pandas as pd
import matplotlib.pyplot as plt

ser = serial.Serial('COM4', 115200, timeout=1)  # ensure COM port is correct and free

with open('speeds.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 15:
        line = ser.readline().decode(errors='ignore')
        if line:
            f.write(line)

ser.close()

# Load CSV file; convert columns to numeric
df = pd.read_csv('speeds.txt', names=['time_ms', 'left_speed', 'right_speed'])
df['time_ms'] = pd.to_numeric(df['time_ms'], errors='coerce')
df['left_speed'] = pd.to_numeric(df['left_speed'], errors='coerce')
df['right_speed'] = pd.to_numeric(df['right_speed'], errors='coerce')
df = df.dropna()

# Convert time to seconds
df['time_s'] = df['time_ms'] / 1000.0

# Define target speeds according to your Arduino phases
# Phase 1: 0–5 s, Phase 2: 5–10 s, Phase 3: 10+ s
df['left_target'] = 0
df['right_target'] = 0
df.loc[df['time_s'] < 5, 'left_target'] = 100
df.loc[df['time_s'] < 5, 'right_target'] = 100
df.loc[(df['time_s'] >= 5) & (df['time_s'] < 10), 'left_target'] = 50
df.loc[(df['time_s'] >= 5) & (df['time_s'] < 10), 'right_target'] = 50

# Plot actual vs target speeds
plt.figure(figsize=(10,5))
plt.plot(df['time_s'], df['left_speed'], label='Left Actual', color='blue')
plt.plot(df['time_s'], df['right_speed'], label='Right Actual', color='red')
plt.plot(df['time_s'], df['left_target'], '--', label='Left Target', color='cyan')
plt.plot(df['time_s'], df['right_target'], '--', label='Right Target', color='orange')

plt.xlabel('Time (s)')
plt.ylabel('Speed (cm/s)')
plt.title('Wheel Speeds vs Time with Targets')
plt.legend()
plt.grid(True)
plt.savefig("speed_test.png");
plt.show()
