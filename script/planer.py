import rospy
import numpy as np

class TrapezoidalPlanner:
    def __init__(self, max_speed=0.5, accel=0.3):
        self.max_speed = max_speed
        self.accel = accel
        self.target_distance = 0
        self.start_time = None
        self.start_pos = 0

    def update_target(self,start, distance):
        self.target_distance = distance
        self.start_time = rospy.Time.now().to_sec()
        self.start_pos = start

    def get_reference(self):
        if self.target_distance == 0:
            return 0, 0  # 目标位置, 目标速度

        t_now = rospy.Time.now().to_sec() - self.start_time
        sign = np.sign(self.target_distance)
        distance = abs(self.target_distance)

        # 计算加速段、匀速段时间
        t_acc = self.max_speed / self.accel
        d_acc = 0.5 * self.accel * t_acc**2
        if distance < 2*d_acc:  # 无法达到最大速度
            t_acc = np.sqrt(distance / self.accel)
            t_total = 2 * t_acc
            speed_profile = self.accel * t_acc
        else:
            d_const = distance - 2*d_acc
            t_const = d_const / self.max_speed
            t_total = 2*t_acc + t_const
            speed_profile = self.max_speed

        # 生成参考信号
        if t_now < t_acc:
            target_vel = self.accel * t_now
            target_pos = 0.5 * self.accel * t_now**2
        elif t_now < t_total - t_acc:
            target_vel = speed_profile
            target_pos = d_acc + speed_profile*(t_now - t_acc)
        elif t_now < t_total:
            target_vel = speed_profile - self.accel*(t_now - (t_total - t_acc))
            target_pos = distance - 0.5*self.accel*(t_total - t_now)**2
        else:
            target_vel = 0
            target_pos = distance

        return sign*(self.start_pos + target_pos), sign*target_vel