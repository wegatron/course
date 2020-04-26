# 实现voxel滤波，并加载数据集中的文件进行验证

import open3d as o3d 
import os
import numpy as np
from pyntcloud import PyntCloud


# 功能：对点云进行voxel滤波
# 输入：
#     point_cloud：输入点云
#     leaf_size: voxel尺寸
#     centroid: 是否使用centroid方法
def voxel_filter(point_cloud, leaf_size, centroid=False):
    filtered_points = []
    # 作业3
    # 屏蔽开始
    pcs = point_cloud.values
    pts_num = len(pcs)
    max_pt = np.amax(pcs, axis=0)
    min_pt = np.amin(pcs, axis=0)
    voxel_size = (np.ceil((max_pt - min_pt)/leaf_size)).astype(np.int64)

    buffer_size = 1000
    raw_inds = np.floor_divide(pcs - min_pt, leaf_size).astype(np.int) % buffer_size

    # 预计算跨度
    vs = [voxel_size[0], (voxel_size[0]%buffer_size) * (voxel_size[1]%buffer_size) % buffer_size]

    # 0 : bin中落入的点的数量
    # 1,2,3 : sum x,y,z
    cpts = np.zeros([buffer_size, 4], dtype=np.float64)
    bin_id = np.full([buffer_size, 3], -1, dtype=np.int)
    for i in range(0, pts_num):
        ind = (raw_inds[i, 0] + raw_inds[i, 1]*vs[0] + raw_inds[i, 2] * vs[1]) % buffer_size
        if np.array_equal(raw_inds[i], bin_id[ind]) or cpts[ind, 0] < 1e-3:
            if centroid:  # 记录点数和sum
                cpts[ind, 0] = cpts[ind, 0] + 1
                cpts[ind, 1:] = cpts[ind, 1:] + pcs[i]
            else:  # 使用最新的
                cpts[ind, 0] = 1
                cpts[ind, 1:] = pcs[i]
            bin_id[ind] = raw_inds[i, :]
        else:  # 将该bin中的点置换出来
            filtered_points.append(cpts[ind, 1:]/cpts[ind, 0])
            cpts[ind, 0] = 1
            cpts[ind, 1:] = pcs[i]
            bin_id[ind] = raw_inds[i, :]

    for ind in range(0, buffer_size):
        if cpts[ind, 0] > 1e-3:
            filtered_points.append(cpts[ind, 1:] / cpts[ind, 0])
    # 屏蔽结束

    # 把点云格式改成array，并对外返回
    filtered_points = np.array(filtered_points, dtype=np.float64)
    return filtered_points


def main():
    # # 从ModelNet数据集文件夹中自动索引路径，加载点云
    # cat_index = 10 # 物体编号，范围是0-39，即对应数据集中40个物体
    # root_dir = '/Users/renqian/cloud_lesson/ModelNet40/ply_data_points' # 数据集路径
    # cat = os.listdir(root_dir)
    # filename = os.path.join(root_dir, cat[cat_index],'train', cat[cat_index]+'_0001.ply') # 默认使用第一个点云
    # point_cloud_pynt = PyntCloud.from_file(file_name)

    # 加载自己的点云文件
    file_name = "/media/wegatron/data/data/ModelNet/data/bed_0517.off"
    point_cloud_pynt = PyntCloud.from_file(file_name)

    # 转成open3d能识别的格式
    point_cloud_o3d = point_cloud_pynt.to_instance("open3d", mesh=False)
    # o3d.visualization.draw_geometries([point_cloud_o3d]) # 显示原始点云

    # 调用voxel滤波函数，实现滤波
    filtered_cloud = voxel_filter(point_cloud_pynt.points, 6.0, False)
    point_cloud_o3d.points = o3d.utility.Vector3dVector(filtered_cloud)
    # 显示滤波后的点云
    o3d.visualization.draw_geometries([point_cloud_o3d])


if __name__ == '__main__':
    main()
