#include <map.h>
#include "gaussian_newton_method.h"

const double GN_PI = 3.1415926;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if(angle > GN_PI)
        angle -= 2*GN_PI;
    else if(angle < -GN_PI)
        angle += 2*GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T  << cos(vec(2)),-sin(vec(2)),vec(0),
            sin(vec(2)), cos(vec(2)),vec(1),
            0,           0,     1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt,Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0),pt(1),1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0),tmp_pt(1));
}



//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
                                std::vector<Eigen::Vector2d> laser_pts,
                                double resolution)
{
    map_t* map = map_alloc();


    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for(int i = 0; i < laser_pts.size();i++)
    {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i],Trans);

        int cell_x,cell_y;
        cell_x = MAP_GXWX(map,tmp_pt(0));
        cell_y = MAP_GYWY(map,tmp_pt(1));

        map->cells[MAP_INDEX(map,cell_x,cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map,0.5);

    return map;
}


/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
/*
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, const Eigen::Vector2d& coords)
{
    const int c_ind_x = MAP_GXWX(map,coords(0));
    const int c_ind_y = MAP_GYWY(map,coords(1));

    // check if coords in map, make sure c_ind +- 1 is safe
    if(c_ind_x <1 || c_ind_x >= map->size_x-1 || c_ind_y<1 || c_ind_y>=map->size_y-1)
        return Eigen::Vector3d::Zero();

    double factors[2] = {
       (coords[0]-MAP_WXGX(map, c_ind_x))/map->resolution,
       (coords[1]-MAP_WYGY(map, c_ind_y))/map->resolution
    };

    double intensities[4] = {0,0,0,0};
    int index = MAP_INDEX(map, c_ind_x, c_ind_y);
    intensities[0] = map->cells[index].score;

    ++index;
    intensities[1] = map->cells[index].score;

    index += map->size_x-1;
    intensities[2] = map->cells[index].score;

    ++index;
    intensities[3] = map->cells[index].score;

    double dx1 = (intensities[0] - intensities[1])/map->resolution;
    double dx2 = (intensities[2] - intensities[3])/map->resolution;

    double dy1 = (intensities[0] - intensities[2])/map->resolution;
    double dy2 = (intensities[1] - intensities[3])/map->resolution;
    double xFacInv = (1.0f - factors[0]);
    double yFacInv = (1.0f - factors[1]);

    return Eigen::Vector3d(
      ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
      ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
      -((dx1 * xFacInv) + (dx2 * factors[0])),
      -((dy1 * yFacInv) + (dy2 * factors[1])));
}
*/

Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, const Eigen::Vector2d& coords)
{
    // find the closest four cell
    const int c_ind_x = MAP_GXWX(map,coords(0));
    const int c_ind_y = MAP_GYWY(map,coords(1));

    // check if coords in map, make sure c_ind +- 1 is safe
    if(c_ind_x <1 || c_ind_x >= map->size_x-1 || c_ind_y<1 || c_ind_y>=map->size_y-1)
        return Eigen::Vector3d::Zero();

    const double w_x = MAP_WXGX(map, c_ind_x);
    const double w_y = MAP_WYGY(map, c_ind_y);

    // find the closest four points' index
    int index_x0, index_y0;
    double x0, y0;
    if(coords[0] > w_x)
    {
      x0 = w_x;
      index_x0 = c_ind_x;
    } else {
      index_x0 = c_ind_x - 1;
      x0 = w_x - map->resolution;
    }

    if(coords[1] > w_y) {
      y0 = w_y;
      index_y0 = c_ind_y;
    } else {
      index_y0 = c_ind_y-1;
      y0 = w_y - map->resolution;
    }

    double factors[2] = { (coords[0] - x0)/map->resolution, (coords[1]-y0)/map->resolution };
    int index_00 = MAP_INDEX(map, index_x0, index_y0);
    double intensities[4] = {map->cells[index_00].score, map->cells[index_00+1].score,
                             map->cells[index_00+map->size_x].score, map->cells[index_00+map->size_x+1].score};

    double dx1 = (intensities[0] - intensities[1])/map->resolution;
    double dx2 = (intensities[2] - intensities[3])/map->resolution;

    double dy1 = (intensities[0] - intensities[2])/map->resolution;
    double dy2 = (intensities[1] - intensities[3])/map->resolution;
    double xFacInv = (1.0f - factors[0]);
    double yFacInv = (1.0f - factors[1]);

    return Eigen::Vector3d(
      ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
      ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
      -((dx1 * yFacInv) + (dx2 * factors[1])),
      -((dy1 * xFacInv) + (dy2 * factors[0])));
}

Eigen::Affine2d getTransformForState(const Eigen::Vector3d& transVector)
{
  return Eigen::Translation2d(transVector[0], transVector[1]) * Eigen::Rotation2Dd(transVector[2]);
}


/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* map, Eigen::Vector3d pose,
                        std::vector<Eigen::Vector2d>& laser_pts,
                        Eigen::Matrix3d& H, Eigen::Vector3d& dTr)
{
    int size = laser_pts.size();

    Eigen::Affine2d transform(getTransformForState(pose));

    double sinRot = sin(pose[2]);
    double cosRot = cos(pose[2]);

    H = Eigen::Matrix3d::Zero();
    dTr = Eigen::Vector3d::Zero();

    for (int i = 0; i < size; ++i) {

      const Eigen::Vector2d& currPoint = laser_pts[i];

      Eigen::Vector3d transformedPointData(InterpMapValueWithDerivatives(map, transform * currPoint));

      double funVal = 1.0f - transformedPointData[0];

      dTr[0] += transformedPointData[1] * funVal;
      dTr[1] += transformedPointData[2] * funVal;

      double rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] + (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);

      dTr[2] += rotDeriv * funVal;

      H(0, 0) += transformedPointData[1] * transformedPointData[1];
      H(1, 1) += transformedPointData[2] * transformedPointData[2];
      H(2, 2) += rotDeriv * rotDeriv;

      H(0, 1) += transformedPointData[1] * transformedPointData[2];
      H(0, 2) += transformedPointData[1] * rotDeriv;
      H(1, 2) += transformedPointData[2] * rotDeriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);
}


/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t*map,Eigen::Vector3d& init_pose,std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 20;
    Eigen::Vector3d now_pose = init_pose;
    Eigen::Matrix3d H;
    Eigen::Vector3d b;
#if 1
    for(int i = 0; i < maxIteration;i++)
    {
        //TODO
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);
        if(H.determinant() < 1e-6) {
            std::cerr << "Unable to solve H is invertible!!!" << std::endl;
            break;
        }
        auto delta_t = H.inverse() * b;
        now_pose += delta_t;
        //if(delta_t.x() < 0.0017 && delta_t.y() < 0.01 && delta_t.z() <0.01) break;
        //END OF TODO
    }
#endif
    init_pose = now_pose;
}
