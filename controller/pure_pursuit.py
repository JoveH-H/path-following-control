import numpy as np
import math


class CU:
    '''
    纯跟踪控制器
    '''

    def __init__(self, ld):
        '''
        纯跟踪控制器初始化
        参数:
            ld: 预瞄距离
        '''
        self.ld = ld

    def get_deltat(self, model, alpha):
        '''
        获取前轮转角
        参数:
            model: 被控模型
            alpha: 期望角度
        '''
        delta = math.atan2(2.0 * model.l * np.sin(alpha - model.theta) / self.ld, 1)
        if delta > np.pi / 5:
            delta = np.pi / 5
        elif delta < -np.pi / 5:
            delta = -np.pi / 5
        return delta


if __name__ == "__main__":
    input("纯跟踪控制器，任意键退出")
