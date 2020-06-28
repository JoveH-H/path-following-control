import numpy as np
import math


class CU:
    '''
    纯跟踪控制器
    '''

    def __init__(self, ld):
        '''
        初始化预瞄距离
        '''
        self.ld = ld

    def get_deltat(self, model, alpha):
        '''
        获取前轮转角
        '''
        delta = math.atan2(2.0 * model.l * np.sin(alpha - model.theta) / self.ld, 1)
        return delta


if __name__ == "__main__":
    input("纯跟踪控制器，任意键退出")
