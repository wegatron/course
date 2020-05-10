# Object detection homework
## ground removal
这里参考论文《Gaussian-Process-Based Real-Time Ground Segmentation
for Autonomous Land Vehicles》使用高斯过程的方法来滤除地面点, 详情:[高斯过程的文档](gaussian_process_ground_segment.pdf).

## segmentation
使用了DBScan进行点云分割.
![segmented_res](segmented_res.png)