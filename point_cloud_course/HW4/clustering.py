# 文件功能：
#     1. 从数据集中加载点云数据
#     2. 从点云数据中滤除地面点云
#     3. 从剩余的点云中提取聚类

import numpy as np
import os
import struct
from sklearn import cluster, datasets, mixture, linear_model
from itertools import cycle, islice
import matplotlib.pyplot as plt
import open3d as o3d
import wegatron_ground_seg
from mpl_toolkits.mplot3d import Axes3D


# 功能：从kitti的.bin格式点云文件中读取点云
# 输入：
#     path: 文件路径
# 输出：
#     点云数组
def read_velodyne_bin(path):
    '''
    :param path:
    :return: homography matrix of the point cloud, N*3
    '''
    pc_list = []
    with open(path, 'rb') as f:
        content = f.read()
        pc_iter = struct.iter_unpack('ffff', content)
        for idx, point in enumerate(pc_iter):
            pc_list.append([point[0], point[1], point[2]])
    return np.asarray(pc_list, dtype=np.float32)


# 功能：从点云文件中滤除地面点
# 输入：
#     data: 一帧完整点云
# 输出：
#     segmengted_cloud: 删除地面点之后的点云
def ground_segmentation(data):
    # 作业1
    # 屏蔽开始
    seg = wegatron_ground_seg.gaussian_process_ground_seg('/media/wegatron/data/workspace/zsw_course/point_cloud_course/HW4/config.yaml')
    labels = seg.segment(data)
    return labels
    #segmengted_cloud = data[np.argwhere(labels == 0).flatten()]
    # ground_pts = o3d.geometry.PointCloud()
    # ground_pts.points = o3d.utility.Vector3dVector(data[np.argwhere(labels == 1).flatten()])
    # ground_pts.paint_uniform_color([0,0,255])

    # other_pts = o3d.geometry.PointCloud()
    # other_pts.points = o3d.utility.Vector3dVector(data[np.argwhere(labels == 0).flatten()])
    # other_pts.paint_uniform_color([0,255,0])
    # o3d.visualization.draw_geometries([ground_pts] + [other_pts])
    # 屏蔽结束
    #print('segmented data points num:', segmengted_cloud.shape[0])
    #return segmengted_cloud


# 功能：从点云中提取聚类
# 输入：
#     data: 点云（滤除地面后的点云）
# 输出：
#     clusters_index： 一维数组，存储的是点云中每个点所属的聚类编号（参考上一章内容容易理解）
def clustering(data):
    # 作业2
    # 屏蔽开始
    dbscan = cluster.DBSCAN(eps=0.3)
    cluster_index = dbscan.fit_predict(data)
    print(cluster_index)
    # 屏蔽结束
    return cluster_index


# 功能：显示聚类点云，每个聚类一种颜色
# 输入：
#      data：点云数据（滤除地面后的点云）
#      cluster_index：一维数组，存储的是点云中每个点所属的聚类编号（与上同）
def plot_clusters(data, cluster_index, ground_points):
    num_clu = np.max(cluster_index)
    colors = ['#377eb8', '#ff7f00', '#4daf4a', '#f781bf', '#a65628', '#984ea3', '#999999', '#e41a1c', '#dede00']
    num_color = len(colors)
    draw_pts_list = []
    for i in range(num_clu):
        cur_color = colors[i%num_color]
        cur_pts = o3d.geometry.PointCloud()
        cur_pts.points = o3d.utility.Vector3dVector(data[np.argwhere(cluster_index == i).flatten()])
        cc = [int(cur_color[j:j + 2], 16)*0.0039 for j in (1, 3, 5)]
        cur_pts.paint_uniform_color(cc)
        draw_pts_list.append(cur_pts)
    ground_pts = o3d.geometry.PointCloud()
    ground_pts.points = o3d.utility.Vector3dVector(ground_points)
    ground_pts.paint_uniform_color([0,0,0])
    draw_pts_list.append(ground_pts)
    o3d.visualization.draw_geometries(draw_pts_list)
    # ax = plt.figure().add_subplot(111, projection = '3d')
    # colors = np.array(list(islice(cycle(['#377eb8', '#ff7f00', '#4daf4a',
    #                                          '#f781bf', '#a65628', '#984ea3',
    #                                          '#999999', '#e41a1c', '#dede00']),
    #                                   int(max(cluster_index) + 1))))
    # colors = np.append(colors, ["#000000"])
    # ax.scatter(data[:, 0], data[:, 1], data[:, 2], s=2, color=colors[cluster_index])
    # plt.show()


def main():
    root_dir = '/media/wegatron/data/data/kitti/object'  # 数据集路径
    cat = os.listdir(root_dir)
    cat = cat[1:]
    iteration_num = len(cat)

    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        print('clustering pointcloud file:', filename)

        origin_points = read_velodyne_bin(filename)
        labels = ground_segmentation(origin_points)
        # 这里做了一些改动, 为了地面点和其他分好的点一起可视化
        segmented_points = origin_points[np.argwhere(labels == 0).flatten()]
        ground_points = origin_points[np.argwhere(labels == 1).flatten()]
        cluster_index = clustering(segmented_points)
        plot_clusters(segmented_points, cluster_index, ground_points)


if __name__ == '__main__':
    main()
