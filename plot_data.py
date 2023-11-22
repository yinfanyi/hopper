import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import os



# 读取最新的CSV文件
folder_path = "./temp/sensor_data"
files = os.listdir(folder_path)
filtered_files = [file for file in files if file.startswith('sensor_data_') and file.endswith('.csv')]
sorted_files = sorted(filtered_files, key=lambda x: os.path.getmtime(os.path.join(folder_path, x)), reverse=True)
newest_file = sorted_files[0]
filename = folder_path + "/" +newest_file
data = pd.read_csv(filename)

indices = [i for i, x in enumerate(data['Time']) if abs(x - 1) < 0.01]
print(indices)
max_jump_height = data[data.index > 20]['Jump Height'].max()
print(max_jump_height)

# 画图
plt.figure(1, figsize=(10, 8))
# plt.clf()
# plt.subplot(511)
plt.plot(data['Time'], data['Jump Height'], label='Jump Height')
# plt.plot(data['Time'], data['Potential Energy'], label='Potential Energy')
# plt.plot(data['Time'], data['Total Energy'], label='Total Energy')
plt.xlabel('Time')
plt.ylabel('Height')
plt.legend()

# plt.figure(2)
# plt.subplot(512)
# # plt.clf()
# plt.plot(data['Time'], data['Spring Length'], label='spring_length')
# plt.plot(data['Time'], data['Jump Height'], label='height')
# plt.plot(data['Time'], data['Hip Pos Sensor Data'], label='hip_pos_sensor')
# plt.xlabel('Time')
# plt.ylabel('length')
# plt.legend()
# # plt.draw()

# # plt.figure(3)
# plt.subplot(513)
# # plt.clf()
# # plt.plot(times, hip_vel_sensor_datas, label='hip_vel_sensor')
# plt.plot(data['Time'], data['Velocity Sensor Data'], label='velocity_sensor')
# plt.xlabel('Time')
# plt.ylabel('velocity_sensor')
# plt.legend()

# plt.subplot(514)
# plt.plot(data['Time'], data['Acceleration Sensor Data'], label='acceleration_sensor')
# plt.xlabel('Time')
# plt.ylabel('acceleration_sensor')
# plt.legend()
# # plt.draw()


# plt.subplot(515)
# plt.plot(data['Time'], data['Ground Contact Force_X'], label='ground_force_x')
# plt.plot(data['Time'], data['Ground Contact Force_Y'], label='ground_force_y')
# plt.plot(data['Time'], data['Ground Contact Force_Z'], label='ground_force_z')
# plt.xlabel('Time')
# plt.ylabel('ground_force')
# plt.legend()
    
# plt.savefig(filename)
plt.show()
# plt.savefig(filename)
