# 文件功能： 实现 K-Means 算法

import numpy as np
import random


class K_Means(object):
    # k是分组数；tolerance‘中心点误差’；max_iter是迭代次数
    def __init__(self, n_clusters=2, tolerance=0.0001, max_iter=300):
        self.k_ = n_clusters
        self.tolerance_ = tolerance
        self.max_iter_ = max_iter
        self.center_ = np.empty([1], dtype=float)

    def fit(self, data):
        # 作业1
        # 屏蔽开始
        pt_num = data.shape[0]
        self.center_ = data[random.sample(range(pt_num), self.k_)]
        cluster = np.empty([pt_num], dtype=int)

        for iter in range(self.max_iter_):
            old_center = np.copy(self.center_)
            dis = np.zeros([pt_num, self.k_], dtype=np.float)
            for k in range(self.k_):
                dc = data - self.center_[k]
                dis[:, k] = np.linalg.norm(data - self.center_[k], axis=1)
            cluster = np.argmin(dis, axis=1)

            for k in range(self.k_):
                c_pts = (np.argwhere(cluster == k)).flatten()
                assert c_pts.size != 0
                self.center_[k] = np.mean(data[c_pts])
            move = np.linalg.norm(old_center - self.center_, axis=1)
            if np.max(move) <= self.tolerance_:
                break
        print(self.center_)
        # 屏蔽结束

    def predict(self, p_datas):
        # 作业2
        # 屏蔽开始
        pt_num = p_datas.shape[0]
        dis = np.zeros([pt_num, self.k_], dtype=np.float)
        for k in range(self.k_):
            dis[:, k] = np.linalg.norm(p_datas - self.center_[k], axis=1)
        result = np.argmin(dis, axis=1)
        # 屏蔽结束
        return result


if __name__ == '__main__':
    x = np.array([[1, 2], [1.5, 1.8], [5, 8], [8, 8], [1, 0.6], [9, 11]])
    k_means = K_Means(n_clusters=2)
    k_means.fit(x)

    cat = k_means.predict(x)
    print(cat)

