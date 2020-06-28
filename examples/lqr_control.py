import sys
sys.path.append("..")
from model import particle
from controller import lqr

import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # 开启interactive mode
    plt.ion()

    # 设置图画尺寸
    plt.figure(figsize=(10, 3))

    # 设置参考轨迹
    PATH_LENGTH, COUNT = 15, 1000
    refer_path = np.zeros((COUNT, 2))
    refer_path[:, 0] = np.linspace(0, PATH_LENGTH, COUNT)
    refer_head = np.zeros(COUNT)

    # 生成sin参考轨迹
    refer_path[:, 1] = 0.2 * np.sin(refer_path[:, 0]) - 0.1

    # 画规划路径
    plt.plot(refer_path[:, 0], refer_path[:, 1], '--y', linewidth=3.0)

    # X坐标，Y坐标，偏转角
    X, Y, THETA = 0, 0.2, 0
    # 速度，车轮间距，间隔时间
    V, L, DT = 0.15, 0.4, 0.1
    # 初始化质点模型
    robot = particle.MODEL(X, Y, THETA,  V, L, DT)

    Q = np.eye(4)
    Q[0, 0] = 1.5  # 横向误差权重
    Q[1, 1] = 0.3  # 横向误差率权重
    Q[2, 2] = 0.5  # 航向误差权重
    Q[3, 3] = 0.1  # 航向误差率权重
    R = 0.1  # 输入量权重

    # 配置线性二次型调节控制器（这里并非完全准确的LQR）
    controller = lqr.CU(Q, R)

    for _ in range(COUNT - 1):
        # 求解车身中心最近的路径点及其差距
        ind, e = controller.calc_nearest_index(robot, refer_path[:, 0], refer_path[:, 1])

        dx, dy = refer_path[ind+1] - refer_path[ind]
        pd = dy / dx

        ddx, _ = (0.5 * (refer_path[ind+1] - refer_path[ind-1])) ** 2
        _, ddy = refer_path[ind+1] - 2 * refer_path[ind] + refer_path[ind-1]
        pdd = ddy / ddx

        # 获取前轮转角
        delta = controller.get_deltat(robot, e, pd, pdd)

        # 更新机器状态
        robot.update(0, delta)

        # 显示实际运行路径
        robot.plot_dynamic_path(3, 1)

        # 间隔
        plt.pause(0.01)