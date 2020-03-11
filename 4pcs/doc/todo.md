## Idea
根据我们的应用场景添加额外的约束, 这里我们假定激光帧P与local map Q的Transform只有三个自由度: x, y, yaw.

### 准备工作
- [x] 为每一帧数据拆解出local map, 然后用算法去做配准
- [x] congruent transform visualization
- [ ] 为了保证不误删求解空间, 准备如下数据集进行验证:
> laser frame P $\to$ local map Q
> ground truth transform: T
场景:
> 望景, 变电站, IDC
从而检查搜索过程中是否会搜到T附近的解, 在加入筛选条件后不会误删.


## 算法
## 4pcs

### GICP

### Fast Global Registration
open3d