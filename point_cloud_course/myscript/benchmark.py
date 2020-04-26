# 对数据集中的点云，批量执行构建树和查找，包括kdtree和octree，并评测其运行时间

import random
import math
import numpy as np
import time
import os
import struct
import wegatron_octree

import octree as octree
import kdtree as kdtree
from result_set import KNNResultSet, RadiusNNResultSet
import open3d as o3d


def read_velodyne_bin(path):
    '''
    :param path:
    :return: homography matrix of the point cloud, N*3
    '''
    pc_list = []
    with open(path, 'rb') as f:
        content = f.read()
        pc_iter = struct.iter_unpack('ffff', content)
        count = 0
        for idx, point in enumerate(pc_iter):
            pc_list.append([point[0], point[1], point[2]])
            count = count + 1
            if count == 10:
                break
    return np.asarray(pc_list, dtype=np.float32).T


def visualize_result(point_cloud_db, knn_result_set=None, radius_result_set=None, show=True):
    if not show:
        return
    #生成原点云
    point_cloud = o3d.geometry.PointCloud()  # 要显示的原始点云
    knn_result_cloud = o3d.geometry.PointCloud()  # 要显示的Knn搜索结果
    radius_result_cloud = o3d.geometry.PointCloud()  # 要显示的Knn搜索结果
    point_cloud.points = o3d.utility.Vector3dVector(point_cloud_db)
    if knn_result_set is not None:
        result_point = [[] for i in range(knn_result_set.capacity)]
        result_point_color = np.zeros([knn_result_set.capacity, 3], int)
        for i in range(knn_result_set.capacity):
            result_point[i] = point_cloud_db[knn_result_set.dist_index_list[i].index]
        knn_result_cloud.points = o3d.utility.Vector3dVector(np.asarray(result_point))
        knn_result_cloud.colors = o3d.utility.Vector3dVector(result_point_color)  # 定义点云的颜色(黑色)

    if radius_result_set is not None:
        if radius_result_set.count == 0:
            return
        result_point = [[] for i in range(radius_result_set.count)]
        result_point_color = np.asarray([[0,0,1] for i in range(radius_result_set.count)])
        for i in range(radius_result_set.count):
            result_point[i] = point_cloud_db[radius_result_set.dist_index_list[i].index]
        radius_result_cloud.points = o3d.utility.Vector3dVector(np.asarray(result_point))
        radius_result_cloud.colors = o3d.utility.Vector3dVector(result_point_color)  # 定义点云的颜色(蓝色)
    o3d.visualization.draw_geometries([knn_result_cloud] + [radius_result_cloud]+[point_cloud])


def main():
    # configuration
    leaf_size = 350
    min_extent = 0.0001
    k = 8
    radius = 1

    root_dir = '/media/wegatron/data/workspace/zsw_course/point_cloud_course/HW2/test_data'  # 数据集路径
    cat = os.listdir(root_dir)
    iteration_num = len(cat)

    print("octree --------------")
    construction_time_sum = 0
    knn_time_sum = 0
    radius_time_sum = 0
    brute_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        begin_t = time.time()
        root = octree.octree_construction(db_np, leaf_size, min_extent)
        construction_time_sum += time.time() - begin_t

        query = db_np[0,:]

        begin_t = time.time()
        nn_result_set = KNNResultSet(capacity=k)
        octree.octree_knn_search(root, db_np, nn_result_set, query)
        knn_time_sum += time.time() - begin_t

        begin_t = time.time()
        result_set = RadiusNNResultSet(radius=radius)
        octree.octree_radius_search_fast(root, db_np, result_set, query)
        radius_time_sum += time.time() - begin_t

        begin_t = time.time()
        diff = np.linalg.norm(np.expand_dims(query, 0) - db_np, axis=1)
        nn_idx = np.argsort(diff)
        nn_dist = diff[nn_idx]
        brute_time_sum += time.time() - begin_t

        # check result is right
        nn_ind = nn_result_set.nearest_nn_index()
        radius_ind = result_set.nearest_radius_index()
        if not np.all(radius_ind == nn_idx[:result_set.count]):
            print('check radius nn False!')
        if not np.all(nn_ind == nn_idx[:nn_result_set.count]):
            print('check knn False')
    print("Octree: build %.3f, knn %.3f, radius %.3f, brute %.3f" % (construction_time_sum*1000/iteration_num,
                                                                     knn_time_sum*1000/iteration_num,
                                                                     radius_time_sum*1000/iteration_num,
                                                                     brute_time_sum*1000/iteration_num))

    print("kdtree --------------")
    construction_time_sum = 0
    knn_time_sum = 0
    radius_time_sum = 0
    brute_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        db_np = read_velodyne_bin(filename)

        begin_t = time.time()
        root = kdtree.kdtree_construction(db_np, leaf_size)
        construction_time_sum += time.time() - begin_t

        query = db_np[0,:]

        begin_t = time.time()
        result_set = KNNResultSet(capacity=k)
        kdtree.kdtree_knn_search(root, db_np, result_set, query)
        knn_time_sum += time.time() - begin_t

        begin_t = time.time()
        result_set = RadiusNNResultSet(radius=radius)
        kdtree.kdtree_radius_search(root, db_np, result_set, query)
        radius_time_sum += time.time() - begin_t

        begin_t = time.time()
        diff = np.linalg.norm(np.expand_dims(query, 0) - db_np, axis=1)
        nn_idx = np.argsort(diff)
        nn_dist = diff[nn_idx]
        brute_time_sum += time.time() - begin_t
    print("Kdtree: build %.3f, knn %.3f, radius %.3f, brute %.3f" % (construction_time_sum * 1000 / iteration_num,
                                                                     knn_time_sum * 1000 / iteration_num,
                                                                     radius_time_sum * 1000 / iteration_num,
                                                                     brute_time_sum * 1000 / iteration_num))


def test_main():
    # configuration
    leaf_size = 350
    min_extent = 0.0001
    k = 8
    radius = 1

    root_dir = '/media/wegatron/data/workspace/zsw_course/point_cloud_course/HW2/test_data'  # 数据集路径
    cat = os.listdir(root_dir)
    iteration_num = len(cat)

    print("octree --------------")
    construction_time_sum = 0
    knn_time_sum = 0
    radius_time_sum = 0
    brute_time_sum = 0
    for i in range(iteration_num):
        filename = os.path.join(root_dir, cat[i])
        # TODO remove the test condition in read velodyne
        db_np = read_velodyne_bin(filename)
        #print(db_np)
        oct = wegatron_octree.octree(db_np, 10, 1.0)
        query_pt = np.array([0,0,0])
        inds, dist = oct.search_knn(query_pt, 6)
        print(query_pt)
        # print(dist)


if __name__ == '__main__':
    test_main()
    # main()