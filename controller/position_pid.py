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

    def limit(self, out, limit=np.pi/5):
        '''
        输出限幅
        参数:
            out: 输出
            limit: 积分量限幅
        '''
        if out > limit:
            out = limit
        elif out < -limit:
            out = -limit
        return out

    def update_e(self, e, ei_limit=np.pi/6):
        '''
        更新偏差
        参数:
            e: 测量值与给定值之间的差
            ei_limit: 积分量限幅
        '''
        now_e = copy.deepcopy(e)
        self.ed = now_e - self.ep
        self.ei += now_e
        self.ei = self.limit(self.ei, ei_limit)
        self.ep = now_e

    def get_u(self, u_limit=np.pi/5):
        '''
        更新给定值
        返回:
            u: 给定值
            u_limit: 给定值限幅
        '''
        u = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        u = self.limit(u, u_limit)
        return u


if __name__ == "__main__":
    input("位置式PID控制器，任意键退出")
