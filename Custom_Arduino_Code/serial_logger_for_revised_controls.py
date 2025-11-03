import serial, time
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import filtfilt, butter

ser = serial.Serial('COM4', 115200, timeout=1)  # ensure COM port is correct and free
ser.flushInput()

with open('speeds.txt', 'w') as f:
    start = time.time()
    while time.time() - start < 20:
        line = ser.readline().decode(errors='ignore')
        if line:
            f.write(line)

ser.close()

# Load CSV file; convert columns to numeric
df = pd.read_csv('speeds.txt', names=['left_speed', 'right_speed', 'left_target', 'right_target', 'left_control', 'right_control', 'time_us'])
df['time_us'] = pd.to_numeric(df['time_us'], errors='coerce')
df['left_speed'] = pd.to_numeric(df['left_speed'], errors='coerce')
df['right_speed'] = pd.to_numeric(df['right_speed'], errors='coerce')
#b, a = butter(6, 15.0/(100.0/2.0), btype="low")
#df['left_speed'] = filtfilt(b, a, df['left_speed'])
#df['right_speed'] = filtfilt(b, a, df['right_speed'])
df['left_target'] = pd.to_numeric(df['left_target'], errors='coerce')
df['right_target'] = pd.to_numeric(df['right_target'], errors='coerce')
df['left_control'] = pd.to_numeric(df['left_control'], errors='coerce')
df['right_control'] = pd.to_numeric(df['right_control'], errors='coerce')
df = df.dropna()

# Convert time to seconds
df['time_s'] = df['time_us'] / 1000000.0
df['left_control'] = df['left_control'] * 6.0/255.0
df['right_control'] = df['right_control'] * 6.0/255.0


# Plot actual vs target speeds
plt.subplot(2, 1, 1);
plt.plot(df['time_s'], df['left_speed'], label='Left Speed', color='blue')
plt.plot(df['time_s'], df['right_speed'], label='Right Speed', color='red')
plt.plot(df['time_s'], df['left_target'], '--', label='Left Target', color='cyan')
plt.plot(df['time_s'], df['right_target'], '--', label='Right Target', color='orange')


plt.title(f'Motor Control Performance Test')
plt.ylabel('Speed (cm/s)')
plt.legend()
plt.grid(True)

# Plot control signals
plt.subplot(2, 1, 2);
plt.plot(df['time_s'], df['left_control'], label='Left Control', color='blue')
plt.plot(df['time_s'], df['right_control'], label='Right Control', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Motor Voltage')


plt.legend()
plt.grid(True)
plt.tight_layout(pad=0.0, w_pad=0.5, h_pad=0.5)
plt.gcf().set_size_inches(16, 9)
plt.savefig("test.png", dpi=600);
plt.show()
