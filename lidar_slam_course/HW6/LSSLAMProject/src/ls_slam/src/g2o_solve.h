#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <gaussian_newton.h>

class vertex_2d : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setToOriginImpl() override
    {
        _estimate.setZero();
    }

    void oplusImpl(const double *update) override {
        _estimate += Eigen::Vector3d(update);
    }

    bool read(std::istream &in) {}

    bool write(std::ostream &out) const {}
};

class edge_2d : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, vertex_2d, vertex_2d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void computeError() override;
    void linearizeOplus() override;

    bool read(std::istream &in) override {}
    bool write(std::ostream &out) const override {}
};

void g2o_solve(std::vector<Eigen::Vector3d>& Vertexs,
                                   std::vector<pa6_gn::Edge>& Edges);



