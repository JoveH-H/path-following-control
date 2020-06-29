import numpy as np
import math


class CU:
    '''
    前轮反馈控制器（Stanley）
    '''

    def __init__(self, k):
        '''
        纯跟踪控制器初始化
        参数:
            k: 增益参数 需满足∣−ke/v∣<=1
        '''
        self.k = k

    def get_deltat(self, model, e, ctheta):
        '''
        获取前轮转角
        参数:
            model: 被控模型
            e: 路径上离被控模型最近点和被控模型的横向位置偏差
            ctheta: 路径上离被控模型最近点的切线角
        '''
        delta = math.atan2(-self.k * e / model.v, 1.0) - (model.theta - ctheta)
        if delta > np.pi / 5:
            delta = np.pi / 5
        elif delta < -np.pi / 5:
            delta = -np.pi / 5
        return delta


if __name__ == "__main__":
    input("Stanley控制器，任意键退出")
