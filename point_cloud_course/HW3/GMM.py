# 文件功能：实现 GMM 算法

import numpy as np
from numpy import *
import pylab
import random,math

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from scipy.stats import multivariate_normal
plt.style.use('seaborn')


def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the
    ellipse patch artist.
    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.
    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    if ax is None:
        ax = plt.gca()

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2 * nstd * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip


class GMM(object):
    def __init__(self, n_clusters, max_iter=50):
        self.n_clusters = n_clusters
        self.max_iter = max_iter
        self.pi = np.empty([n_clusters], dtype=float)
        self.mu = np.empty([n_clusters], dtype=float)
        self.sigma = np.empty([n_clusters, 2, 2], dtype=float)
    
    def fit(self, data):
        # 作业3
        # 屏蔽开始
        pts_num = data.shape[0]
        self.mu = data[random.sample(range(pts_num), self.n_clusters)]
        #self.mu = [[0, -1], [6, 0], [0, 9]]
        for i in range(self.n_clusters):
            self.sigma[i] = np.identity(2)
        self.pi[:] = 1.0/self.n_clusters
        gamma_znk = np.empty([pts_num, self.n_clusters], np.float)
        for itr in range(self.max_iter):
            for k in range(self.n_clusters):
                gamma_znk[:, k] = self.pi[k] * multivariate_normal.pdf(data, mean=self.mu[k], cov=self.sigma[k])
            gamma_znk = gamma_znk/gamma_znk.sum(axis=1).reshape(-1, 1)
            for k in range(self.n_clusters):
                gznk = gamma_znk[:, k].reshape(-1, 1)
                nk = np.sum(gznk)
                inv_nk = 1.0/nk
                wdata = gznk * data
                self.mu[k] = np.sum(wdata, axis=0) * inv_nk
                self.pi[k] = nk / self.n_clusters
                cd = (data - self.mu[k])
                self.sigma[k] = np.dot(cd.T, gznk * cd) * inv_nk
        # 屏蔽结束
    
    def predict(self, data):
        # 屏蔽开始
        pts_num = data.shape[0]
        gamma_znk = np.empty([pts_num, self.n_clusters], dtype=np.float)
        for k in range(self.n_clusters):
            gamma_znk[:, k] = multivariate_normal.pdf(data, mean=self.mu[k], cov=self.sigma[k, :, :])
        result = np.argmax(gamma_znk, axis=1)
        return result
        # 屏蔽结束


# 生成仿真数据
def generate_X(true_Mu, true_Var):
    # 第一簇的数据
    num1, mu1, var1 = 400, true_Mu[0], true_Var[0]
    X1 = np.random.multivariate_normal(mu1, np.diag(var1), num1)
    # 第二簇的数据
    num2, mu2, var2 = 600, true_Mu[1], true_Var[1]
    X2 = np.random.multivariate_normal(mu2, np.diag(var2), num2)
    # 第三簇的数据
    num3, mu3, var3 = 1000, true_Mu[2], true_Var[2]
    X3 = np.random.multivariate_normal(mu3, np.diag(var3), num3)
    # 合并在一起
    X = np.vstack((X1, X2, X3))
    # 显示数据
    # plt.figure(figsize=(10, 8))
    # plt.axis([-10, 15, -5, 15])
    # plt.scatter(X1[:, 0], X1[:, 1], s=5)
    # plt.scatter(X2[:, 0], X2[:, 1], s=5)
    # plt.scatter(X3[:, 0], X3[:, 1], s=5)
    # plt.show()
    return X


if __name__ == '__main__':
    # 生成数据
    true_Mu = [[0.5, 0.5], [5.5, 2.5], [1, 7]]
    true_Var = [[1, 3], [2, 2], [6, 2]]
    X = generate_X(true_Mu, true_Var)

    gmm = GMM(n_clusters=3, max_iter=300)
    gmm.fit(X)
    cat = gmm.predict(X)
    X1 = X[(np.argwhere(cat == 0)).flatten()]
    X2 = X[(np.argwhere(cat == 1)).flatten()]
    X3 = X[(np.argwhere(cat == 2)).flatten()]
    plt.figure(figsize=(10, 8))
    plt.axis([-10, 15, -5, 15])
    plt.scatter(X1[:, 0], X1[:, 1], s=5)
    plt.scatter(X2[:, 0], X2[:, 1], s=5)
    plt.scatter(X3[:, 0], X3[:, 1], s=5)
    for k in range(gmm.n_clusters):
        plot_args = {'fc': 'None', 'lw': 2, 'edgecolor': '#0000FF', 'alpha': 0.5}
        plot_cov_ellipse(gmm.sigma[k], gmm.mu[k], **plot_args)
    plt.show()
    # 初始化

    

