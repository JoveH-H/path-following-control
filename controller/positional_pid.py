import numpy as np
import copy


class CU:
    '''
    位置式PID控制器
    '''

    def __init__(self, kp, ki, kd):
        '''
        位置式PID控制器初始化
        参数:
            kp: 比例系数
            ki: 积分系数
            kd: 微分系数
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ep = 0.0
        self.ei = 0.0
        self.ed = 0.0

    def update_e(self, e):
        '''
        更新偏差
        参数:
            e: 测量值与给定值之间的差
        '''
        self.ed = e - self.ep
        self.ei += e
        self.ep = copy.deepcopy(e)

    def get_u(self):
        '''
        更新给定值
        返回:
            u: 给定值
        '''
        u = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        if u > np.pi / 5:
            u = np.pi / 5
        elif u < -np.pi / 5:
            u = -np.pi / 5
        return u


if __name__ == "__main__":
    input("位置式PID控制器，任意键退出")
