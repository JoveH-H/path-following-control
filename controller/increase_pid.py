import numpy as np
import copy


class CU:
    '''
    增量式PID控制器
    '''

    def __init__(self, kp, ki, kd):
        '''
        增量式PID控制器初始化
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
        self.e1 = 0.0
        self.e2 = 0.0

    def limit(self, out, limit=np.pi/18):
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

    def update_e(self, e):
        '''
        更新偏差
        参数:
            e: 测量值与给定值之间的差
        '''
        now_e = copy.deepcopy(e)
        self.ep = now_e - self.e1
        self.ei = now_e
        self.ed = now_e - 2 * self.e1 + self.e2
        self.e2 = self.e1
        self.e1 = now_e

    def get_ut(self, ut_limit=np.pi/18):
        '''
        更新给定值变化量
        返回:
            ut: 给定值变化量
        '''
        ut = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        ut = self.limit(ut, ut_limit)
        return ut


if __name__ == "__main__":
    input("位置式PID控制器，任意键退出")
