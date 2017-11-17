#!/usr/bin/env python
import numpy as np


class EKF:
    def __init__(self):
        self.acc = np.zeros(3)
        self.gyro = np.zeros(3)

        # EKF
        # R
        self.R = np.zeros((4, 4))
        for i in range(4):
            self.R[i, i] = 0.01
        # Q
        self.Q = np.zeros((3, 3))
        for i in range(3):
            self.Q[i, i] = 0.1
        # S
        self.S = np.zeros((4, 4))
        for i in range(4):
            self.S[i, i] = 1

        self.buf_len = 10
        self.acc_buf = np.zeros((self.buf_len, 3))
        self.gyro_buf = np.zeros((self.buf_len, 3))
        self.acc_std = np.zeros(3)
        self.gyro_std = np.zeros(3)
        self.gyro_abs = 100
        
        self.buf_cnt = 0

    def buf_data(self, gyro, acc):
        self.gyro_buf[self.buf_cnt % self.buf_len] = gyro
        self.acc_buf[self.buf_cnt % self.buf_len] = acc
        if self.buf_cnt >= self.buf_len:
            self.gyro_std = np.std(self.gyro_buf, axis=0)
            self.acc_std = np.std(self.acc_buf, axis=0)
        self.gyro_abs = np.sum(np.abs(gyro))
        self.buf_cnt += 1

    def filter(self, x, S, gyro, z, dt):
        acc = z
        self.buf_data(gyro, acc)
        z = z / np.linalg.norm(z)
        z = z.reshape((3, 1))

        delta_t = dt # 0.05
        # G
        G = np.array([[0, -gyro[0], -gyro[1], -gyro[2]],
                      [gyro[0], 0, gyro[2], -gyro[1]],
                      [gyro[1], -gyro[2], 0, gyro[0]],
                      [gyro[2], gyro[1], -gyro[0], 0]]) * delta_t * 0.5
        # h
        h = -np.array([[2 * (x[1][0] * x[3][0] - x[0][0] * x[2][0])],
                      [2 * (x[2][0] * x[3][0] + x[0][0] * x[1][0])],
                      [1 - 2 * (x[1][0]**2 + x[2][0]**2)]])
        #print h.ravel(), z.ravel()
        # predict
        x = np.dot(np.identity(4) + G, x)
        S = np.dot(np.dot(G, S),G.transpose()) + self.R

        #update
        if True: # np.max(self.gyro_std) < 10 and np.max(self.gyro_std) < 10:
            H = -np.array([[-x[2][0], x[3][0], -x[0][0], x[1][0]],
                          [x[1][0], x[0][0], x[3][0], x[2][0]],
                          [x[0][0], -x[1][0], -x[2][0], x[3][0]]]) * 2
            S_e = np.dot(H, S.dot(H.transpose())) + self.Q
            K = np.dot(S.dot(H.transpose()), np.linalg.inv(S_e))

            x = x + K.dot(z - h)
            S = (np.identity(4) - K.dot(H)).dot(S)
            x = x / np.linalg.norm(x)
        return x, S

    def init_filter(self, gyro, acc):
        self.buf_data(gyro, acc)
        if self.buf_cnt > self.buf_len:
            S = self.S
            acc = acc / np.linalg.norm(acc)
            cross = np.cross(np.array([0, 0, -1]), acc)
            theta = np.arcsin(np.linalg.norm(cross))
            x = np.zeros(4)
            x[0] = np.cos(theta)
            x[1:] = cross * np.sin(theta)
            x = x.reshape((4, 1))
            return x, S
        else:
            return None

