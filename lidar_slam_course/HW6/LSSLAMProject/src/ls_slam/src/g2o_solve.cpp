#include "g2o_solve.h"
#include <chrono>
#include <vector>


void edge_2d::computeError()
{
    const vertex_2d * v0 = static_cast<const vertex_2d*>(_vertices[0]);
    const vertex_2d * v1 = static_cast<const vertex_2d*>(_vertices[1]);
    Eigen::Vector3d xi = v0->estimate();
    Eigen::Vector3d xj = v1->estimate();
    Eigen::Vector3d z = _measurement;

    const Eigen::Matrix2d Ri = pa6_gn::angle2rmat(xi[2]);
    const Eigen::Matrix2d Rj = pa6_gn::angle2rmat(xj[2]);
    const Eigen::Matrix2d Rz = pa6_gn::angle2rmat(z[2]);
    const Eigen::Vector2d ti = xi.block<2,1>(0,0);
    const Eigen::Vector2d tj = xj.block<2,1>(0,0);
    const Eigen::Vector2d tz = z.block<2,1>(0,0);

    _error[2] = xj[2] - xi[2] - z[2];
    if(_error[2] > M_PI) _error[2] -= 2*M_PI;
    else if(_error[2] < -M_PI) _error[2] += 2*M_PI;

    _error.block<2,1>(0,0) =
            Rz.transpose() * (Ri.transpose() * (tj - ti) - tz);
}

void edge_2d::linearizeOplus()
{
    const vertex_2d * v0 = static_cast<const vertex_2d*>(_vertices[0]);
    const vertex_2d * v1 = static_cast<const vertex_2d*>(_vertices[1]);
    Eigen::Vector3d xi = v0->estimate();
    Eigen::Vector3d xj = v1->estimate();
    Eigen::Vector3d z = _measurement;
    const Eigen::Matrix2d Ri = pa6_gn::angle2rmat(xi[2]);
    const Eigen::Matrix2d Rj = pa6_gn::angle2rmat(xj[2]);
    const Eigen::Matrix2d Rz = pa6_gn::angle2rmat(z[2]);
    const Eigen::Vector2d ti = xi.block<2,1>(0,0);
    const Eigen::Vector2d tj = xj.block<2,1>(0,0);
    const Eigen::Vector2d tz = z.block<2,1>(0,0);

    Eigen::Matrix2d rmat_df_theta;
    rmat_df_theta << -sin(xi[2]), -cos(xi[2]),
            cos(xi[2]), -sin(xi[2]);
    _jacobianOplusXi.block<2,2>(0,0) = -Rz.transpose() * Ri.transpose();
    _jacobianOplusXi.block<2,1>(0, 2) = Rz.transpose() * rmat_df_theta.transpose() * (tj - ti);
    _jacobianOplusXi(2,0) = 0; _jacobianOplusXi(2,1) = 0; _jacobianOplusXi(2,2) = -1;

    _jacobianOplusXj.setZero(); _jacobianOplusXj(2,2) = 1;
    _jacobianOplusXj.block<2,2>(0,0)=Rz.transpose() * Ri.transpose();
}

void g2o_solve(std::vector<Eigen::Vector3d> &Vertexs, std::vector<pa6_gn::Edge> &Edges) {

    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    // add vertex
    std::vector<vertex_2d*> vs;
    for(int i=0; i<Vertexs.size(); ++i)
    {
        vertex_2d *cur_v = new vertex_2d;
        cur_v->setEstimate(Vertexs[i]);
        cur_v->setId(i);
        optimizer.addVertex(cur_v);
        if(i == 0) cur_v->setFixed(true);
        vs.emplace_back(cur_v);
    }

    // add edge
    for(int i=0; i<Edges.size(); ++i)
    {
        edge_2d *cur_e = new edge_2d;
        int idxi = Edges[i].xi;
        int idxj = Edges[i].xj;
        cur_e->setInformation(Edges[i].infoMatrix);
        cur_e->setVertex(0, optimizer.vertices()[idxi]);
        cur_e->setVertex(1, optimizer.vertices()[idxj]);
        cur_e->setMeasurement(Edges[i].measurement);
        optimizer.addEdge(cur_e);
        cur_e->setId(i);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(20);

    for(int i=0; i<Vertexs.size(); ++i)
    {
        Vertexs[i] = vs[i]->estimate();
    }
}
