## 目标
Implement ICP, NDT Refere to PCL, 并做优化加速.

transformation_validation.h
> validate the correctness of a transformation found through TransformationEstimation. 
> The output is in the form of a score or a confidence measure

## ICP过程
1. Eigen::Autodiff (deprecated)  ----> 框架代码
    data set
2. 梯度推导 for [x, y, z, 四元素]
3. 带有kernel的梯度推导