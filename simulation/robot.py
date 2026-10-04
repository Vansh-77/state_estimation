import math

class Robot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        
        self.v = 0.0
        self.w = 0.0
        
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.max_accel = 1.0
        self.max_angular_accel = 2.0

        self.accel = 0.0
        self.angular_accel = 0.0  
        
        
    def update(self, dt):
        old_v = self.v
        old_w = self.w

        v_error = self.v_cmd - self.v
        max_dv = self.max_accel * dt

        if abs(v_error) <= max_dv:
            self.v = self.v_cmd
        else:
            self.v += math.copysign(max_dv, v_error)

        w_error = self.w_cmd - self.w
        max_dw = self.max_angular_accel * dt

        if abs(w_error) <= max_dw:
            self.w = self.w_cmd
        else:
            self.w += math.copysign(max_dw, w_error)

        self.accel = (self.v - old_v) / dt
        self.angular_accel = (self.w - old_w) / dt

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
    def command_profile(self,t):
        if t < 2:
            return 0.0, 0.0

        elif t < 5:
            return 0.5, 0.0

        elif t < 8:
            return 0.0, 0.5

        elif t < 11:
            return 0.2, 0.0

        else:
            return 0.0, 0.0