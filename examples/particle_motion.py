import os
import sys
sys.path.append(os.path.join(
    os.path.dirname(__file__), os.path.pardir).replace('\\', '/'))
from model import particle

import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # 开启interactive mode
    plt.ion()

    # 设置图画尺寸
    plt.figure(figsize=(10, 3))

    # 设置参考轨迹
    PATH_LENGTH, COUNT = 1, 100
    refer_path = np.zeros((COUNT, 2))
    refer_path[:, 0] = np.linspace(0, PATH_LENGTH, COUNT)

    # 画规划路径
    plt.plot(refer_path[:, 0], refer_path[:, 1], '--y', linewidth=3.0)

    # X坐标，Y坐标，偏转角
    X, Y, THETA = 0, 0, 0
    # 速度，车轮间距，间隔时间
    V, L, DT = 0.150, 0.40, 0.020
    # 初始化质点模型
    robot = particle.MODEL(X, Y, THETA,  V, L, DT)

    for i in range(COUNT):
        # 方向盘转角以余弦周期性变化
        robot.update(0, np.cos(i / 6.0))
        # 显示实际运行路径
        robot.plot_dynamic_path(0.3, 0.01)
        # 间隔
        plt.pause(0.02)
