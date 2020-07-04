import os
import sys
sys.path.append(os.path.join(
    os.path.dirname(__file__), os.path.pardir).replace('\\', '/'))
from model import particle
from controller import stanley

from scipy.spatial import KDTree
import matplotlib.pyplot as plt
import numpy as np
import math


if __name__ == "__main__":
    # 开启interactive mode
    plt.ion()

    # 设置图画尺寸
    plt.figure(figsize=(10, 3))

    # 设置参考轨迹
    PATH_LENGTH, COUNT = 15, 200
    refer_path = np.zeros((COUNT, 2))
    refer_path[:, 0] = np.linspace(0, PATH_LENGTH, COUNT)

    # 生成sin参考轨迹
    refer_path[:, 1] = 0.2 * np.sin(refer_path[:, 0]) - 0.1
    refer_tree = KDTree(refer_path)

    # 画规划路径
    plt.plot(refer_path[:, 0], refer_path[:, 1], '--y', linewidth=3.0)

    # X坐标，Y坐标，偏转角
    X, Y, THETA = 0, 0.2, 0
    # 速度，车轮间距，间隔时间
    V, L, DT = 0.15, 0.4, 0.1
    # 初始化质点模型
    robot = particle.MODEL(X, Y, THETA,  V, L, DT)

    # 配置增益参数k和stanley控制器
    K = -0.6
    controller = stanley.CU(K)

    # 初始化设备当前位置量和路径上离被控模型最近点编号及其辅助变量
    robot_state = np.zeros(2)
    ind, pind = 0, 0

    for _ in range(COUNT):
        # 获取当前位置
        robot_state[0], robot_state[1] = robot.x, robot.y

        # 计算路径上离被控模型最近点
        _, ind = refer_tree.query(robot_state)
        if ind < pind:
            ind = pind
        else:
            pind = ind

        # 计算路径上离被控模型最近点和被控模型的偏差角
        dx, dy = refer_path[ind] - robot_state
        alpha = math.atan2(dy, dx)

        # 计算路径上离被控模型最近点和被控模型的横向位置偏差
        dist = np.linalg.norm(robot_state - refer_path[ind])
        e = np.sign(np.sin(alpha - robot.theta)) * dist

        # 计算路径上离被控模型最近点的切线角
        ndx, ndy = refer_path[ind + 1] - refer_path[ind]
        ctheta = math.atan2(ndy, ndx)

        # 更新前轮转角
        delta = controller.get_deltat(robot, e, ctheta)

        # 更新机器状态
        robot.update(0, delta)

        # 显示实际路况
        robot.plot_dynamic_path(3, 1)

        # 间隔
        plt.pause(0.01)
