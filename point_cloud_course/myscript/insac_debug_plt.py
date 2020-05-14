import matplotlib.pyplot as plt
import numpy as np

with open("/home/wegatron/tmp/debug_insac.txt", "r") as file:
    i = 0
    model_pts = np.empty([0]) # r, height
    sig_pts = np.empty([0]) # r, height, predict, uncertainty
    for line in file.readlines():
        #print(line)
        if i == 0:
            model_vals = [float(i) for i in line.split()]
            model_pts = np.array(model_vals)
            model_pts = model_pts.reshape(-1, 2)
            print("model vals:", model_pts)
            i = 1
        else:
            buffer_vals = [float(i) for i in line.split()]
            sig_pts = np.array(buffer_vals)
            sig_pts = sig_pts.reshape(-1, 4)
            print("predict vals:", sig_pts)
            i = 0

            # do plot
            train_X = model_pts[:,0]
            train_y = model_pts[:,1]
            test_X = sig_pts[:,0]
            test_y = sig_pts[:,1]
            predict_y = sig_pts[:,2]
            uncertainty = sig_pts[:,3]

            plt.figure()
            plt.title("gaussian insac")
            plt.fill_between(test_X.ravel(), predict_y + uncertainty, predict_y - uncertainty, alpha=0.1)
            plt.plot(test_X, predict_y, label="predict")
            plt.scatter(train_X, train_y, label="train", c="red", marker="x")
            plt.scatter(test_X, test_y, label="test", c="green", marker="*")
            plt.legend()
plt.show()
input("Press Enter to continue...")


# from scipy.optimize import minimize


# class GPR:
#     def __init__(self, optimize=True):
#         self.is_fit = False
#         self.train_X, self.train_y = None, None
#         self.params = {"l": 16, "sigma_sf": 1, "sigma_sn": 0.05}
#         self.optimize = optimize
#
#     def fit(self, X, y):
#         # store train data
#         self.train_X = np.asarray(X)
#         self.train_y = np.asarray(y)
#         self.is_fit = True
#
#     def predict(self, X):
#         if not self.is_fit:
#             print("GPR Model not fit yet.")
#             return
#
#         X = np.asarray(X)
#         Kff = self.kernel(self.train_X, self.train_X)  # (N, N)
#         Kyy = self.kernel(X, X)  # (k, k)
#         Kfy = self.kernel(self.train_X, X)  # (N, k)
#         Kff_inv = np.linalg.inv(Kff + self.params["sigma_sn"] * np.eye(len(self.train_X)))  # (N, N)
#
#         mu = Kfy.T.dot(Kff_inv).dot(self.train_y)
#         cov = Kyy - Kfy.T.dot(Kff_inv).dot(Kfy)
#         return mu, cov
#
#     def kernel(self, x1, x2):
#         dist_matrix = np.sum(x1 ** 2, 1).reshape(-1, 1) + np.sum(x2 ** 2, 1) - 2 * np.dot(x1, x2.T)
#         return self.params["sigma_sf"] * np.exp(-0.5 / self.params["l"] ** 2 * dist_matrix)
#
#
# with open("/home/wegatron/tmp/debug_insac.txt", "r") as file:
#     i = 0
#     model_pts = np.empty([0]) # r, height
#     sig_pts = np.empty([0]) # r, height, predict, uncertainty
#     for line in file.readlines():
#         #print(line)
#         if i == 0:
#             model_vals = [float(i) for i in line.split()]
#             model_pts = np.array(model_vals)
#             model_pts = model_pts.reshape(-1, 2)
#             print("model vals:", model_pts)
#             i = 1
#         else:
#             buffer_vals = [float(i) for i in line.split()]
#             sig_pts = np.array(buffer_vals)
#             sig_pts = sig_pts.reshape(-1, 4)
#             print("predict vals:", sig_pts)
#             i = 0
#             train_X = np.expand_dims(model_pts[:,0], 1)
#             train_y = np.expand_dims(model_pts[:,1], 1)
#             num_model = model_pts.shape[0]
#             test_X = np.expand_dims(sig_pts[:,0], 1)
#             test_y = sig_pts[:,1]
#             gpr = GPR()
#             gpr.fit(train_X, train_y)
#             mu, cov = gpr.predict(test_X)
#             predict_y = mu.ravel()
#             uncertainty = 0.3*np.sqrt(np.diag(cov))
#             plt.figure()
#             plt.title("l=%.2f sigma_sf=%.2f sigma_sn=%.2f" % (gpr.params["l"], gpr.params["sigma_sf"], gpr.params["sigma_sn"]))
#             plt.fill_between(test_X.ravel(), predict_y + uncertainty, predict_y - uncertainty, alpha=0.1)
#             plt.plot(test_X, predict_y, label="predict")
#             plt.scatter(test_X, test_y, label="train", c="green", marker="o")
#             plt.scatter(train_X, train_y, label="train", c="red", marker="x")
#             plt.legend()
#             plt.show()
#             input("Press Enter to continue...")



