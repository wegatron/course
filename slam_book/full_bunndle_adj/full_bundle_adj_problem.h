#ifndef FULL_BUNDLE_ADJ_PROBLEM_H
#define FULL_BUNDLE_ADJ_PROBLEM_H

#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <ceres/internal/autodiff.h>

#include "g2o/stuff/sampler.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/solvers/structure_only/structure_only_solver.h"



#include "ref/projection.h"

/**
 * calculate camera projection with distortion.
 * @param camera camera params: [0-2] axis_angle, theta * rotation_axis; [3-5] translate; [6-8] f, k1, k2
 * @param point 3d point
 * @return projected pixel
 */
Eigen::Vector2d cam_project_with_distortion(const Eigen::Matrix<double, 9,1> &camera,
                                            const Eigen::Vector3d &point);

/**
 * \brief vertex point for optimization
 */
class vertex_point final : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	vertex_point() {}
	bool read(std::istream& is) override { return false; }
	bool write(std::ostream& os) const override { return false; }
	void setToOriginImpl() override {}

	// for vertex update
	void oplusImpl(const double *update) override {
		Eigen::Vector3d::ConstMapType v(update);
		_estimate += v;
	}
};

/**
 * \brief vertex_camera 9D vector.
 * 3d angle axis
 * 3d translate
 * f, k1, k2
 */
class vertex_camera final : public g2o::BaseVertex<9, Eigen::Matrix<double,9,1>>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	vertex_camera() {}
	bool read(std::istream& is) override { return false; }
	bool write(std::ostream& os) const override { return false; }

	// vertex initial value
	void setToOriginImpl() {}

	// for vertex update
	void oplusImpl(const double *update) override {
		Eigen::Matrix<double,9,1>::ConstMapType v(update, vertex_camera::Dimension);
		_estimate += v;
	}
};

class edge_observation final : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, vertex_camera, vertex_point>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    edge_observation() {}
    bool read(std::istream& is) override { return false; }
    bool write(std::ostream& os) const override { return false; }

    // energy value evaluator
    void computeError() override
    {
      const vertex_camera *cam_v = static_cast<const vertex_camera*>(vertex(0));
      const vertex_point *point_v = static_cast<const vertex_point*>(vertex(1));

      // calculate the error
      auto prediction = cam_project_with_distortion(cam_v->estimate(), point_v->estimate());
      _error = prediction - measurement();
//		( *this ) ( cam_v->estimate().data(), point_v->estimate().data(), _error.data() );
    }


    // using auto diff, here must be template, and unable to use eigen to calculate the residuals
    template<typename T>
    bool operator() (const T * cam, const T *pt, T *residuals) const
	{
        T predictions[2];
        CamProjectionWithDistortion ( cam, pt, predictions );
        residuals[0] = predictions[0] - T ( measurement() ( 0 ) );
        residuals[1] = predictions[1] - T ( measurement() ( 1 ) );
		return true;
	}

	// partial diff of the energy
    void linearizeOplus() override
    {
		const vertex_camera * cam = static_cast<const vertex_camera*> ( vertex ( 0 ) );
		const vertex_point* point = static_cast<const vertex_point*> ( vertex ( 1 ) );

		// here using ceres auto diff
		typedef ceres::internal::AutoDiff<edge_observation, double, vertex_camera::Dimension, vertex_point::Dimension> BalAutoDiff;

		Eigen::Matrix<double, Dimension, vertex_camera::Dimension, Eigen::RowMajor> dError_dCamera;
		Eigen::Matrix<double, Dimension, vertex_point::Dimension, Eigen::RowMajor> dError_dPoint;
		double *parameters[] = { const_cast<double*> ( cam->estimate().data() ), const_cast<double*> ( point->estimate().data() ) };
		double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
		double value[Dimension];
		bool diffState = BalAutoDiff::Differentiate ( *this, parameters, Dimension, value, jacobians );

		// copy over the Jacobians (convert row-major -> column-major)
		if ( diffState )
		{
			_jacobianOplusXi = dError_dCamera;
			_jacobianOplusXj = dError_dPoint;
		}
		else
		{
			assert ( 0 && "Error while differentiating" ); // nice handle
			_jacobianOplusXi.setZero();
			_jacobianOplusXj.setZero();
		}
	}

};

struct observation
{
	int cam_id;
	int pt_id;
	Eigen::Vector2d ob_pixel;
};

class full_bla_problem final
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	full_bla_problem() {}
	int load(const std::string &file_path);

	/**
	 * \brief build the bla problem after load
	 */
	void build_up();

	void solve();

	void update();

	void write_ply(const std::string &file_path);


	/**
	 * Add noise for the input data.
	 * @param rotation_sigma
	 * @param translation_sigma
	 * @param point_sigma
	 */
	void perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);

	// for test
	void cam_rt_test();
private:
	std::vector<Eigen::Matrix<double,9,1>> cameras_; //[0-2] axis_angle, theta * rotation_axis; [3-5] translate; [6-8] f, k1, k2
	std::vector<Eigen::Vector3d> pts_;
	std::vector<observation> obs_;
	g2o::SparseOptimizer optimizer_;
};

#endif //FULL_BUNDLE_ADJ_PROBLEM_H
