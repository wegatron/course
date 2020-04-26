# DSO 第⼆次作业
## 一、代码
1. `error.txt`结果

    ```
    0.0006
    0.0007
    0.0007
    0.0008
    0.0008
    0.0007
    0.0009
    0.0009
    0.0011
    0.0012
    ...
    0.0039
    0.0039
    0.0042
    0.0043
    0.0046
    0.0046
    0.0048
    0.0048
    0.0050
    0.0054
    0.0055
    0.0058
    0.0055
    0.0053
    0.0054
    ```

2. 点数对结果的影响, `grid_size`表示将图像做`grid_size` $\times$ `grid_size`的划分, 在每个`grid`上最多取一个点. 当`grid_size` $\to \infty$等价于选取所有满足fast阈值的点.
    `grid_size`从5到15, error的变化.(点从非常不足到基本足够)
    ![5to15](5_15_static.png)
    
    `grid_size`从10到100, error的变化. error基本稳定, 可见`grid_size=10`是一个比较好的阈值.
    ![10to100](10_100_static.png)

## 二、关键帧筛选策略总结&分析
### ORB-SLAM2
ORB-SLAM使用宽进严出的关键帧筛选策略, 插入关键帧的条件有:
1. 已经间隔足够多的帧都没有插入关键帧了
2. 局部地图空闲时, 放低关键帧间隔阈值
3. 跟踪到的点很少, 快要挂
4. 跟踪地图MapPoints的比例比较少

```c++
// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
//Condition 1c: tracking is weak
const bool c1c =  mSensor!=System::MONOCULAR 
    && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
// Condition 2: Few tracked points compared to reference keyframe. 
// Lots of visual odometry compared to map matches.
const bool c2=((mnMatchesInliers<nRefMatches*thRefRatio||bNeedToInsertClose) && mnMatchesInliers>15);

if((c1a||c1b||c1c)&&c2) {
...
}
```

其中条件4必须满足, 1,2,3至少有一个满足. 关键帧在插入之后, 后续会根据共视关系, 删除掉与其他帧重复过多的帧.

ORB-SLAM的关键帧筛选策略主要基于特征点, 当看到新的特征点时加入关键帧.

### DSO
DSO主要考虑当前帧和前⼀关键帧在点的光流变化，不考虑旋转情况下的光流变化，曝光参数的变化，三者加权相加⼤于１时新建关键帧.
```c++
needToMakeKF = allFrameHistory.size()== 1 ||
setting_kfGlobalWeight*setting_maxShiftWeightT * sqrtf((double)tres[1]) / (wG[0]+hG[0]) +
setting_kfGlobalWeight*setting_maxShiftWeightR * sqrtf((double)tres[2]) / (wG[0]+hG[0]) +
setting_kfGlobalWeight*setting_maxShiftWeightRT * sqrtf((double)tres[3]) / (wG[0]+hG[0]) +
setting_kfGlobalWeight*setting_maxAffineWeight * fabs(logf((float)refToFh[0])) > 1 ||
2*coarseTracker->firstCoarseRMSE < tres[0];
```

DSO只是一个odometry, 因此, 只需要考虑当前跟踪是否需要此帧作为参照. 移动、旋转带来的光流变化和关照变化, 都会影响跟踪, 需要及时加入关键帧. 后续会自动保留最近的关键帧, 将以前的关键帧边缘化掉.