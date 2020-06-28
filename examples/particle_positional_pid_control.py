import sys
sys.path.append("..")
from model import particle
from controller import positional_pid

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
    PATH_LENGTH, COUNT = 15, 1000
    refer_path = np.zeros((COUNT, 2))
    refer_path[:, 0] = np.linspace(0, PATH_LENGTH, COUNT)

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

    # 配置位置式PID控制器参数
    pid = positional_pid.CU(0.8, 0.005, 5)

    refer_tree = KDTree(refer_path)
    robot_state = np.zeros(2)

    for _ in range(COUNT):
        # 获取误差角度
        robot_state[0], robot_state[1] = robot.x, robot.y
        _, ind = refer_tree.query(robot_state)
        _, dy = refer_path[ind] - robot_state
        ndx, ndy = refer_path[ind + 1] - refer_path[ind]
        dx = V * np.cos(math.atan2(ndy, ndx)) * robot.dt
        alpha = math.atan2(dy, dx)
        e = alpha - robot.theta

        # 更新误差
        pid.update_e(e)

        # 输出PID，更新机器状态
        update = pid.get_u() - robot.theta
        if update > np.pi / 3:
            update = np.pi / 3
        elif update < -np.pi / 3:
            update = -np.pi / 3
        robot.update(0, update)

        # 显示实际路况
        robot.plot_dynamic_path(3, 1)

        # 间隔
        plt.pause(0.01)
