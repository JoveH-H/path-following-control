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

    def update_e(self, e):
        '''
        更新偏差
        参数:
            e: 测量值与给定值之间的差
        '''
        self.ed = e - self.ep
        self.ei = e + self.ep
        self.ep = copy.deepcopy(e)

    def get_ut(self):
        '''
        更新给定值变化量
        返回:
            ut: 给定值变化量
        '''
        ut = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        if ut > np.pi / 5:
            ut = np.pi / 5
        elif ut < -np.pi / 5:
            ut = -np.pi / 5
        return ut


if __name__ == "__main__":
    input("位置式PID控制器，任意键退出")
