# HW1
1. 通过两帧间特征点的匹配求解F矩阵,并恢复R,t.
    两个思路: 直接求E, 或者先求F, 再计算E.

2. 不同噪声下, 两种方法的对比
![max_by_noise](doc/EF_by_noise/max.png)
![mean_by_noise](doc/EF_by_noise/mean.png)
![median_by_noise](doc/EF_by_noise/median.png)
![min_by_noise](doc/EF_by_noise/min.png)
![rmse_by_noise](doc/EF_by_noise/rmse.png)
![sse_by_noise](doc/EF_by_noise/sse.png)
![std_by_noise](doc/EF_by_noise/std.png)

3. 特征点分布逐渐变广过程中, 两种方法的对比
![max_by_noise](doc/EF_by_ld_scale/max.png)
![mean_by_noise](doc/EF_by_ld_scale/mean.png)
![median_by_noise](doc/EF_by_ld_scale/median.png)
![min_by_noise](doc/EF_by_ld_scale/min.png)
![rmse_by_noise](doc/EF_by_ld_scale/rmse.png)
![sse_by_noise](doc/EF_by_ld_scale/sse.png)
![std_by_noise](doc/EF_by_ld_scale/std.png)
![tc](doc/EF_by_ld_scale/tc.png)