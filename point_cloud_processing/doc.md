## 当前的问题
使用Lego-Loam可以快速准确的估计高频激光帧之间的Transform.
但是, 运动畸变, 以及激光本身的误差/畸变, 会使得激光帧出现无法完全配准的情况:
一部分配上, 另一部分脱离.
选取什么样的策略, 对激光数据进行滤波, 可以获得更好的地图呢?

## 典型的点云去噪
参考"A review of algorithms for filtering the 3D point cloud", 典型的几种方法
* Normal-based Bilateral Filter (NBF)  [implemented in pcl]
* Weighted Locally Optimal Projection (WLOP) [implemented in cgal]
* Edge Aware Resample (EAR) and L0 minimization [EAR implemented in cgal]
* RMLS, [MLS implemented in pcl]

### RMLS
将Moving Least Square改为 Robust Moving Least Square需要做以下几件事情:
1. verify smooth and not smooth, classification
2. forward search for points near the edge
3. normal estimation, convex and concave check
4. project points to smooth surface
5. project points to edge

### 参考实现
> cgal 点云处理 https://github.com/petteriTeikari/CGAL_pointCloud
> L0 minimization: mesh denosing via L0 mnimization, https://github.com/bldeng/GuidedDenoising