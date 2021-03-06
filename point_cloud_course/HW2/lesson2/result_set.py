# 该文件定义了在树中查找数据所需要的数据结构，类似一个中间件

import copy
import numpy as np


class DistIndex:
    def __init__(self, distance, index):
        self.distance = distance
        self.index = index

    def __lt__(self, other):
        return self.distance < other.distance


class KNNResultSet:
    def __init__(self, capacity=None, inds=None, dist=None):
        if capacity is not None:
            self.capacity = capacity
            self.count = 0
            self.worst_dist = 1e10
            self.dist_index_list = []
            for i in range(capacity):
                self.dist_index_list.append(DistIndex(self.worst_dist, 0))
            self.comparison_counter = 0
        elif inds is not None:
            self.capacity = inds.shape[0]
            self.count = inds.shape[0]
            self.worst_dist = dist[-1]
            self.dist_index_list = []
            for i in range(self.count):
                self.dist_index_list.append(DistIndex(dist[i], inds[i]))

    def size(self):
        return self.count

    def full(self):
        return self.count == self.capacity

    def worstDist(self):
        return self.worst_dist

    def add_point(self, dist, index):
        self.comparison_counter += 1
        if dist > self.worst_dist:
            return

        if self.count < self.capacity:
            self.count += 1

        # 插入排序
        i = self.count - 1
        while i > 0:
            if self.dist_index_list[i-1].distance > dist:
                self.dist_index_list[i] = copy.deepcopy(self.dist_index_list[i-1])
                i -= 1
            else:
                break

        self.dist_index_list[i].distance = dist
        self.dist_index_list[i].index = index
        self.worst_dist = self.dist_index_list[self.capacity-1].distance
        
    def __str__(self):
        output = ''
        for i, dist_index in enumerate(self.dist_index_list):
            output += '%d - %.2f\n' % (dist_index.index, dist_index.distance)
        output += 'In total %d comparison operations.' % self.comparison_counter
        return output

    def nearest_nn_index(self):
        ret = np.empty([self.count], dtype=int)
        for i in range(self.count):
            ret[i] = self.dist_index_list[i].index
        return ret


class RadiusNNResultSet:
    def __init__(self, radius, inds_radius=None, dis=None):
        self.radius = radius
        self.worst_dist = radius
        self.dist_index_list = []
        if inds_radius is not None:
            self.count = inds_radius.shape[0]
            for i in range(self.count):
                self.dist_index_list.append(DistIndex(dis[i], inds_radius[i]))
        else:
            self.count = 0
        self.comparison_counter = 0

    def size(self):
        return self.count

    def worstDist(self):
        return self.radius

    def add_point(self, dist, index):
        self.comparison_counter += 1
        if dist > self.radius:
            return

        self.count += 1
        self.dist_index_list.append(DistIndex(dist, index))

    def __str__(self):
        self.dist_index_list.sort()
        output = ''
        for i, dist_index in enumerate(self.dist_index_list):
            output += '%d - %.2f\n' % (dist_index.index, dist_index.distance)
        output += 'In total %d neighbors within %f.\nThere are %d comparison operations.' \
                  % (self.count, self.radius, self.comparison_counter)
        return output

    def nearest_radius_index(self):
        ret = np.empty([self.count], dtype=int)
        self.dist_index_list.sort()
        for i in range(self.count):
            ret[i] = self.dist_index_list[i].index
        return ret

