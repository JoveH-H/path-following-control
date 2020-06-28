import numpy as np
import copy


class CU:
    '''
    位置式PID控制器
    '''

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ep = 0.0
        self.ei = 0.0
        self.ed = 0.0

    def update_e(self, e):
        self.ed = e - self.ep
        self.ei += e
        self.ep = copy.deepcopy(e)

    def get_u(self):
        u = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed
        if u > np.pi / 3:
            u = np.pi / 3
        elif u < -np.pi / 3:
            u = -np.pi / 3
        return u


if __name__ == "__main__":
    input("位置式PID控制器，任意键退出")
