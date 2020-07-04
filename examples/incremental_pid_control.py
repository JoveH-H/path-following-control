import os
import sys
sys.path.append(os.path.join(
    os.path.dirname(__file__), os.path.pardir).replace('\\', '/'))
from model import particle
from controller import incremental_pid

from scipy.spatial import KDTree
import math
import matplotlib.pyplot as plt
import numpy as np


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

    # 配置增量式PID控制器参数
    pid = incremental_pid.CU(0.8, 0.05, 20)

    # 初始化设备当前位置量
    robot_state = np.zeros(2)

    for _ in range(COUNT):
        # 获取当前位置
        robot_state[0], robot_state[1] = robot.x, robot.y
        _, ind = refer_tree.query(robot_state)

        # 计算当前与路径的横向距离偏差
        _, dy = refer_path[ind] - robot_state

        # 计算下一状态与当前路径x方向的期望距离
        ndx, ndy = refer_path[ind + 1] - refer_path[ind]
        dx = V * np.cos(math.atan2(ndy, ndx)) * robot.dt

        # 计算期望偏转角
        alpha = math.atan2(dy, dx)

        # 计算偏转角偏差
        e = alpha - robot.theta

        # 更新偏差
        pid.update_e(e)

        # 输出PID，更新机器状态
        robot.update(0, pid.get_ut())

        # 显示实际路况
        robot.plot_dynamic_path(3, 1)

        # 间隔
        plt.pause(0.01)
