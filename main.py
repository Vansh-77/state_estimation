from simulation.robot import Robot
from simulation.sensor import Encoder , IMU
import matplotlib.pyplot as plt
import math

robot = Robot()

dt = 0.05

encoder = Encoder()
enc_x = robot.x
enc_y = robot.y
enc_theta = robot.theta

enc_x_values = [enc_x]
enc_y_values = [enc_y]
enc_theta_values = []
enc_w_values = []

x_values = [robot.x]
y_values = [robot.y]
theta_values = []

epos_vals = []

time_values = []
velocity_values = []
acceleration_values = []
angular_velocity_values = []
angular_acceleration_values = []

imu = IMU()
imu_theta = robot.theta
imu_theta_values = []
imu_w_values = []

gyro_error_values = []
enc_error_values = []

fused_w_values = []
fused_theta = robot.theta
fused_theta_values = []
alpha = 0.68

for i in range(420):
    t = i * dt
    time_values.append(t)
    robot.v_cmd, robot.w_cmd = robot.command_profile(t)
    robot.update(dt)
    x_values.append(robot.x)
    y_values.append(robot.y)
    theta_values.append(robot.theta)
    
    vl , vr = encoder.get_enc_data(robot)
    est_v = (vr+vl)/2
    est_w = (vr-vl)/encoder.wheel_base
    
    enc_x = enc_x + est_v*math.cos(enc_theta)*dt
    enc_y = enc_y + est_v*math.sin(enc_theta)*dt
    enc_theta = enc_theta + est_w*dt
    
    ex = robot.x - enc_x
    ey = robot.y - enc_y
    epos = math.sqrt(ex**2 + ey**2)
    epos_vals.append(epos)
    
    gyro, accel = imu.get_imu_data(robot)
    imu_theta += gyro * dt
    imu_theta_values.append(imu_theta)
        
    enc_x_values.append(enc_x)
    enc_y_values.append(enc_y)
    enc_theta_values.append(enc_theta)
    
    velocity_values.append(robot.v)
    acceleration_values.append(robot.accel)
    angular_velocity_values.append(robot.w)
    angular_acceleration_values.append(robot.angular_accel)
    gyro_error = robot.theta - imu_theta
    enc_error = robot.theta - enc_theta
    gyro_error_values.append(gyro_error)
    enc_error_values.append(enc_error)
    enc_w_values.append(est_w)
    imu_w_values.append(gyro)
    fused_w = alpha*gyro + (1-alpha)*est_w
    fused_theta += fused_w*dt 
    fused_theta_values.append(fused_theta)
    fused_w_values.append(fused_w)
    
plt.figure()
plt.plot(x_values, y_values, label="ground truth")
plt.plot(enc_x_values, enc_y_values, label="dead reckoning")
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

plt.figure()

plt.plot(time_values, theta_values, label="Ground Truth")
plt.plot(time_values, imu_theta_values, label="Gyro")
plt.plot(time_values, enc_theta_values, label="Encoder")
plt.plot(time_values, fused_theta_values, label="fused theta")

plt.plot(time_values, enc_error_values, label="encoder error")
plt.plot(time_values, gyro_error_values, label="gyro error")

plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.title("Orientation Estimation")
plt.legend()
plt.grid()

plt.figure()

plt.plot(time_values, enc_w_values, label="Encoder")
plt.plot(time_values, imu_w_values, label="Gyro")
plt.plot(time_values, angular_velocity_values, label="Ground Truth")
plt.plot(time_values, fused_w_values, label="fused w")

plt.xlabel("Time (s)")
plt.ylabel("w (rad/s)")
plt.title("w Estimation")
plt.legend()
plt.grid()


plt.show()