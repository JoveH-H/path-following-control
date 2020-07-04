import os
import sys
sys.path.append(os.path.join(
    os.path.dirname(__file__), os.path.pardir).replace('\\', '/'))
from model import particle
from controller import pure_pursuit

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

    # 画规划路径
    plt.plot(refer_path[:, 0], refer_path[:, 1], '--y', linewidth=3.0)

    # X坐标，Y坐标，偏转角
    X, Y, THETA = 0, 0.2, 0
    # 速度，车轮间距，间隔时间
    V, L, DT = 0.15, 0.4, 0.1
    # 初始化质点模型
    robot = particle.MODEL(X, Y, THETA,  V, L, DT)

    # 配置预瞄距离和纯跟踪控制器
    Ld = 0.5
    controller = pure_pursuit.CU(Ld)

    # 初始化设备当前位置量和预瞄点编号
    robot_state = np.zeros(2)
    ind = 0

    for _ in range(COUNT):
        # 获取当前位置
        robot_state[0], robot_state[1] = robot.x, robot.y

        # 搜索预瞄点
        for i in range(ind, len(refer_path)):
            dist = np.linalg.norm(robot_state-refer_path[i])
            if dist >= Ld:
                ind = i
                break
        dist = np.linalg.norm(robot_state-refer_path[ind])

        # 获取期望角度
        dx, dy = refer_path[ind] - robot_state
        alpha = math.atan2(dy, dx)

        # 更新前轮转角
        delta = controller.get_deltat(robot, alpha)

        # 更新机器状态
        robot.update(0, delta)

        # 显示实际路况
        robot.plot_dynamic_path(3, 1)

        # 间隔
        plt.pause(0.01)
