from simulation.robot import Robot
from simulation.sensor import Encoder , IMU
import matplotlib.pyplot as plt
import math

robot = Robot()

dt = 0.05

encoder = Encoder()
est_x = robot.x
est_y = robot.y
est_theta = robot.theta

est_x_values = [est_x]
est_y_values = [est_y]

x_values = [robot.x]
y_values = [robot.y]

epos_vals = []

time_values = []
velocity_values = []
acceleration_values = []
angular_velocity_values = []
angular_acceleration_values = []

imu = IMU()
imu_theta = robot.theta
imu_theta_values = []

for i in range(420):
    t = i * dt
    time_values.append(t)
    robot.v_cmd, robot.w_cmd = robot.command_profile(t)
    robot.update(dt)
    x_values.append(robot.x)
    y_values.append(robot.y)
    
    vl , vr = encoder.get_enc_data(robot)
    est_v = (vr+vl)/2
    est_w = (vr-vl)/encoder.wheel_base
    
    est_x = est_x + est_v*math.cos(est_theta)*dt
    est_y = est_y + est_v*math.sin(est_theta)*dt
    est_theta = est_theta + est_w*dt
    
    ex = robot.x - est_x
    ey = robot.y - est_y
    epos = math.sqrt(ex**2 + ey**2)
    epos_vals.append(epos)
    
    gyro, accel = imu.get_imu_data(robot)
    imu_theta += gyro * dt
    imu_theta_values.append(imu_theta)
        
    est_x_values.append(est_x)
    est_y_values.append(est_y)
    velocity_values.append(robot.v)
    acceleration_values.append(robot.accel)
    angular_velocity_values.append(robot.w)
    angular_acceleration_values.append(robot.angular_accel)
    
plt.figure()
plt.plot(x_values, y_values, label="ground truth")
plt.plot(est_x_values, est_y_values, label="dead reckoning")
plt.legend()
plt.axis("equal")

# plt.figure()
# plt.plot(time_values, epos_vals)
# plt.xlabel("Time (s)")
# plt.ylabel("Position error (m)")
# plt.title("Dead Reckoning Position Error")
# plt.grid()

plt.figure(figsize=(10, 8))

plt.subplot(2, 2, 1)
plt.plot(time_values, velocity_values)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Robot Velocity")
plt.grid()

plt.subplot(2, 2, 2)
plt.plot(time_values, acceleration_values)
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s²)")
plt.title("Robot Acceleration")
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(time_values, angular_velocity_values)
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Robot Angular Velocity")
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(time_values, angular_acceleration_values)
plt.xlabel("Time (s)")
plt.ylabel("Angular Acceleration (rad/s²)")
plt.title("Robot Angular Acceleration")
plt.grid()

plt.tight_layout()


plt.show()