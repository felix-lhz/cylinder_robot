import math
import time

class PID:
    def __init__(self, P, I, D, current_time=None):
        self.Kp = P  # 比例系数
        self.Ki = I  # 积分系数
        self.Kd = D  # 微分系数

        self.sample_time = 0.01  # 采样时间，默认 0.01 秒
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """清除 PID 控制器的状态"""
        self.set_point = 0.0  # 目标值
        self.P_term = 0.0  # 比例项
        self.I_term = 0.0  # 积分项
        self.D_term = 0.0  # 微分项
        self.last_error = 0.0  # 上一次的误差

        self.output = 0.0  # PID 输出

    def update(self, input,target, current_time=None):
        """根据反馈值更新 PID 输出"""
        error = target - input
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            # 计算比例项
            self.P_term = self.Kp * error

            # 计算积分项
            self.I_term += self.Ki * error * delta_time

            # 计算微分项
            self.D_term = 0.0
            if delta_time > 0:
                self.D_term = self.Kd * delta_error / delta_time

            # 计算 PID 输出
            self.output = self.P_term + self.I_term + self.D_term

            # 保存当前误差和时间
            self.last_error = error
            self.last_time = self.current_time

    def set_setpoint(self, set_point):
        """设置目标值"""
        self.set_point = set_point

    def set_sample_time(self, sample_time):
        """设置采样时间"""
        self.sample_time = sample_time

    def get_output(self):
        """获取当前 PID 输出"""
        return self.output