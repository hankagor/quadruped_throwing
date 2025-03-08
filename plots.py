import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

# Update the path to your actual CSV file
file_path = '/home/hanka/catkin_ws_go1_full/robot_state_20250105_013506.csv'

# Load the CSV file
data = pd.read_csv(file_path)

data['time'] = data['time'] - data['time'].iloc[0]

# Find the time when box_thrown becomes 1
throw_time = data['time'][data['box_thrown'] == 1].iloc[0]

last_pdesx = data['pdesx'].iloc[-1] * np.sqrt(2)
last_pdesz = data['pdesz'].iloc[-1] * np.sqrt(2)
last_vdesx = data['vdesx'].iloc[-1] * np.sqrt(2)
last_vdesz = data['vdesz'].iloc[-1] * np.sqrt(2)

time_interp = np.linspace(data['time'].min(), data['time'].max(), len(data['time']))  # Interpolated time points

interp_pdesx = interp1d(data['time'], data['pdesx'], kind='quadratic')(time_interp)
interp_pdesz = interp1d(data['time'], data['pdesz'], kind='quadratic')(time_interp)
data['vdesx'] = last_vdesx
data['vdesz'] = last_vdesz
# Plot pdesx vs posx
plt.figure(figsize=(10, 6))
plt.plot(data['time'], data['pdesx'], label='$x_{des}$', linewidth=2)
plt.plot(data['time'], data['posx'], label='$x$', linewidth=2)

plt.axvline(x=throw_time, color='r', linestyle=':', label='Box Thrown')
plt.xlabel('Time (s)')
plt.ylabel('Position X (m)')
plt.title('Desired vs Actual Position (X)')
plt.legend()
plt.grid(True)
plt.show()


# Plot vdesx vs velx
plt.figure(figsize=(10, 6))
plt.plot(data['time'], data['vdesx'], label='$v_{xdes}$', linewidth=2)
plt.plot(data['time'], data['velx'], label='$v_x$', linewidth=2)

# plt.axhline(y=last_vdesx, label=f'Last vdesx * sqrt(2)')
plt.axvline(x=throw_time, color='r', linestyle=':', label='Box Thrown')
plt.xlabel('Time (s)')
plt.ylabel('Velocity X (m/s)')
plt.title('Desired vs Actual Velocity (X)')
plt.legend()
plt.grid(True)
plt.show()

# Plot pdesz vs posz
plt.figure(figsize=(10, 6))
plt.plot(data['time'], -data['pdesz'], label='$z_{des}$', linewidth=2)
plt.plot(data['time'], -data['posz'], label='$z$', linewidth=2)

plt.axvline(x=throw_time, color='r', linestyle=':', label='Box Thrown')
plt.xlabel('Time (s)')
plt.ylabel('Position Z (m)')
plt.title('Desired vs Actual Position (Z)')
plt.legend()
plt.grid(True)
plt.show()

data['vdesz'] = last_vdesz
# Plot vdesz vs velz
plt.figure(figsize=(10, 6))
plt.plot(data['time'], -data['vdesz'], label='$v_{zdes}$', linewidth=2)
plt.plot(data['time'], -data['velz'], label='$v_z$', linewidth=2) 

# plt.axhline(y=last_vdesz,  label=f'Last vdesx * sqrt(2)')
plt.axvline(x=throw_time, color='r', linestyle=':', label='Box Thrown')
plt.xlabel('Time (s)')
plt.ylabel('Velocity Z (m/s)')
plt.title('Desired vs Actual Velocity (Z)')
plt.legend()
plt.grid(True)
plt.show()

# # Plot tau0, tau1, tau2
# plt.figure(figsize=(10, 6))
# plt.plot(data['time'], data['tau0'], label='tau0')
# plt.plot(data['time'], data['tau1'], label='tau1')
# plt.plot(data['time'], data['tau2'], label='tau2')
# plt.xlabel('Time (s)')
# plt.ylabel('Torque (Nm)')
# plt.title('Joint Torques (tau0, tau1, tau2)')
# plt.legend()
# plt.grid(True)
# plt.show()

# # Plot tau3, tau4, tau5
# plt.figure(figsize=(10, 6))
# plt.plot(data['time'], data['tau3'], label='tau3')
# plt.plot(data['time'], data['tau4'], label='tau4')
# plt.plot(data['time'], data['tau5'], label='tau5')
# plt.xlabel('Time (s)')
# plt.ylabel('Torque (Nm)')
# plt.title('Joint Torques (tau3, tau4, tau5)')
# plt.legend()
# plt.grid(True)
# plt.show()
