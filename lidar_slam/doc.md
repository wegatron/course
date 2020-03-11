# surfel extract
输入: 点云, 栅格大小, 每个栅格最少包含的点的数量
输出: vector of surfel feature
流程:
1. 遍历每个点, 并计算每个栅格的$\sum_{i=0}^m p_i p^T_i$, 以及每个栅格的中心点$\mu = \frac{1}{m}\sum_{i=0}^m p_i$
2. 计算每个栅格的特征矩阵$E=\frac{1}{m} \sum_{i=0}^m p_i p_i^T - \mu \mu^T$.
3. 对矩阵进行Eigen分解, 求出surfel特征.
