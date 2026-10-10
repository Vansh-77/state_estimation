import numpy as np

class Encoder:
    def __init__(self, wheel_base = 0.3):
        self.wheel_base = wheel_base
        self.vl_bias = 0.002
        self.vr_bias = -0.001
        self.sigma = 0.01
    
    def get_enc_data(self,robot):
        vL = robot.v - self.wheel_base * robot.w/2 + np.random.normal(0,self.sigma) + self.vl_bias
        vR = robot.v + self.wheel_base * robot.w/2 + np.random.normal(0,self.sigma) + self.vr_bias
        return (vL, vR)
        
class IMU:
    def __init__(self):
        self.gyro_bias = 0.005
        self.gyro_sigma = 0.01

        self.accel_bias = 0.02
        self.accel_sigma = 0.05

    def get_imu_data(self, robot):
        gyro = robot.w + self.gyro_bias + np.random.normal(0, self.gyro_sigma)
        accel = robot.accel + self.accel_bias + np.random.normal(0, self.accel_sigma)
        return gyro, accel