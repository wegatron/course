## ORB_SLAM2待整理问题
1. Map, MapPoint, Frame, KeyFrame这些数据结构的意义和作用.
2. opencv中提取orb特征点时是否有考虑到特征点的离散性.


## ORB_SLAM2的一些疑问
1. 关于`ReconstructF`函数, 两个条件判断的问题
    为什么是与
2. stero 亚像素级匹配

3. TrackWithMotionModel, 恒速模型, 不是应该由pnp得到rt吗？