import numpy as np
import math


class CU:
    '''
    纯跟踪控制器
    '''

    def __init__(self, k):
        self.k = k

    def get_deltat(self, model, e):
        '''
        获取前轮转角
        '''
        delta = math.atan2(-self.k * e / model.v, 1.0) - model.theta
        return delta


if __name__ == "__main__":
    input("Stanley控制器，任意键退出")
