import numpy as np
import math
import scipy.linalg as la


class CU:
    '''
    线性二次型调节控制器
    '''

    def __init__(self, Q, R):
        '''
        线性二次型调节控制器初始化
        参数:
            Q: 偏差权重矩阵
            R: 输入量权重矩阵
        '''
        self.x = np.mat(np.zeros((4, 1)))  # 状态变量矩阵
        self.A = np.mat(np.zeros((4, 4)))
        self.B = np.mat(np.zeros((4, 1)))
        self.Q = Q
        self.R = R
        self.pe = 0  # 旧被控模型中心离路径的最近点的差距
        self.pth_e = 0  # 旧路径上离被控模型最近点的切线角和被控设备横摆角的偏差值

    def solve_DARE(self, A, B, Q, R):
        '''
        求解离散时间-代数黎卡提方程
        '''
        P = Q
        maxiter = 200
        eps = 0.01

        for _ in range(maxiter):
            Pn = A.T * P * A - A.T * P * B * la.pinv(R + B.T * P * B) * B.T * P * A + Q
            if (abs(Pn - P)).max() < eps:
                P = Pn
                break
            P = Pn
        return Pn

    def dlqr(self):
        '''
        求解离散时间lqr控制器
        '''
        # 尝试解出ricatti方程
        X = self.solve_DARE(self.A, self.B, self.Q, self.R)

        # 计算LQR增益
        K = la.pinv(self.R + self.B.T * X * self.B) * (self.B.T * X * self.A)

        return K

    def calc_nearest_index(self, model, cx, cy):
        '''
        求解车身中心最近的路径点及其差距
        '''
        dx = [model.x - icx for icx in cx]
        dy = [model.y - icy for icy in cy]

        dist = [abs(math.sqrt(idx ** 2 + idy ** 2))
                for (idx, idy) in zip(dx, dy)]

        error = min(dist)
        ind = dist.index(error)

        if model.y - cy[ind] < 0:
            error = -error

        return ind, error

    def get_deltat(self, model, refer_path):
        '''
        获取前轮转角
        参数:
            model: 被控模型
            refer_path: 参考轨迹
        '''
        # 求解离被控模型中心最近的路径点及其差距
        ind, e = self.calc_nearest_index(model, refer_path[:, 0], refer_path[:, 1])

        # 计算路径上离被控模型最近点的斜率（一阶导数）
        dx, dy = refer_path[ind+1] - refer_path[ind]
        pd = dy / dx

        # 计算路径上离被控模型最近点的斜率变化速度（二阶导数）
        ddx, _ = (0.5 * (refer_path[ind+1] - refer_path[ind-1])) ** 2
        _, ddy = refer_path[ind+1] - 2 * refer_path[ind] + refer_path[ind-1]
        pdd = ddy / ddx

        # 计算路径上离被控模型最近点的切线角
        ctheta = math.atan(pd)

        # 计算路径上离被控模型最近点的切线角和被控设备横摆角的偏差值
        th_e = model.theta - ctheta

        self.A[0, 0] = 1.0
        self.A[0, 1] = model.dt
        self.A[1, 2] = model.v
        self.A[2, 2] = 1.0
        self.A[2, 3] = model.dt

        self.B[3, 0] = model.v / model.l

        self.x[0, 0] = e
        self.x[1, 0] = (e - self.pe) / model.dt
        self.x[2, 0] = th_e
        self.x[3, 0] = (th_e - self.pth_e) / model.dt
        self.pe, self.pth_e = e, th_e

        # 计算曲率
        k = pdd / ((1 + pd ** 2) ** 1.5)

        # 计算状态反馈调节器增益
        K = self.dlqr()

        # 前馈控制输入量
        delta_ff = math.atan(model.l * k)

        # 最优的状态反馈前轮转角
        delta_fd = float(-K * self.x)

        delta = delta_ff + delta_fd
        if delta > np.pi / 5:
            delta = np.pi / 5
        elif delta < -np.pi / 5:
            delta = -np.pi / 5
        return delta


if __name__ == "__main__":
    input("线性二次型调节控制器，任意键退出")
