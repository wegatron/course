#ifndef REMOVE_SMALL_ISLAND_H
#define REMOVE_SMALL_ISLAND_H

#include "shared.h"

void remove_small_island(std::vector<gr::Point3D> &points, const float radius, const int num_pts, const std::string &output_vtk);

void output_res(std::vector<gr::Point3D> &set1, std::vector<gr::Point3D> &set2);

#endif //REMOVE_SMALL_ISLAND_H