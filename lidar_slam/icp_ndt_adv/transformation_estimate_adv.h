#ifndef TRANSFORMATION_ESTIMATE_ADV_H
#define TRANSFORMATION_ESTIMATE_ADV_H

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/warp_point_rigid.h>
#include <pcl/registration/distances.h>

namespace pcl
{
	namespace registration
	{
		template <typename PointSource, typename PointTarget, typename MatScalar = float>
		class TransformationEstimationLM_adv : public pcl::registration::TransformationEstimation<PointSource, PointTarget, MatScalar>
		{
			typedef pcl::PointCloud<PointSource> PointCloudSource;
			typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
			typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

			typedef pcl::PointCloud<PointTarget> PointCloudTarget;

			typedef PointIndices::Ptr PointIndicesPtr;
			typedef PointIndices::ConstPtr PointIndicesConstPtr;

		public:

			typedef boost::shared_ptr<TransformationEstimationLM_adv<PointSource, PointTarget, MatScalar> > Ptr;
			typedef boost::shared_ptr<const TransformationEstimationLM_adv<PointSource, PointTarget, MatScalar> > ConstPtr;

			typedef Eigen::Matrix<MatScalar, Eigen::Dynamic, 1> VectorX;
			typedef Eigen::Matrix<MatScalar, 4, 1> Vector4;
			typedef typename TransformationEstimationLM_adv<PointSource, PointTarget, MatScalar>::Matrix4 Matrix4;

			/** \brief Constructor. */
			TransformationEstimationLM_adv();

			/** \brief Copy constructor.
			  * \param[in] src the TransformationEstimationLM object to copy into this
			  */
			TransformationEstimationLM_adv(const TransformationEstimationLM_adv &src) :
				tmp_src_(src.tmp_src_),
				tmp_tgt_(src.tmp_tgt_),
				tmp_idx_src_(src.tmp_idx_src_),
				tmp_idx_tgt_(src.tmp_idx_tgt_),
				warp_point_(src.warp_point_)
			{};

			/** \brief Copy operator.
			  * \param[in] src the TransformationEstimationLM object to copy into this
			  */
			TransformationEstimationLM_adv&
				operator = (const TransformationEstimationLM_adv &src)
			{
				tmp_src_ = src.tmp_src_;
				tmp_tgt_ = src.tmp_tgt_;
				tmp_idx_src_ = src.tmp_idx_src_;
				tmp_idx_tgt_ = src.tmp_idx_tgt_;
				warp_point_ = src.warp_point_;
			}

			/** \brief Destructor. */
			virtual ~TransformationEstimationLM_adv() {};

			/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
			  * \param[in] cloud_src the source point cloud dataset
			  * \param[in] cloud_tgt the target point cloud dataset
			  * \param[out] transformation_matrix the resultant transformation matrix
			  */
			inline void
				estimateRigidTransformation(
					const pcl::PointCloud<PointSource> &cloud_src,
					const pcl::PointCloud<PointTarget> &cloud_tgt,
					Matrix4 &transformation_matrix) const;

			/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
			  * \param[in] cloud_src the source point cloud dataset
			  * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
			  * \param[in] cloud_tgt the target point cloud dataset
			  * \param[out] transformation_matrix the resultant transformation matrix
			  */
			inline void
				estimateRigidTransformation(
					const pcl::PointCloud<PointSource> &cloud_src,
					const std::vector<int> &indices_src,
					const pcl::PointCloud<PointTarget> &cloud_tgt,
					Matrix4 &transformation_matrix) const;

			/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
			  * \param[in] cloud_src the source point cloud dataset
			  * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
			  * \param[in] cloud_tgt the target point cloud dataset
			  * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from
			  * \a indices_src
			  * \param[out] transformation_matrix the resultant transformation matrix
			  */
			inline void
				estimateRigidTransformation(
					const pcl::PointCloud<PointSource> &cloud_src,
					const std::vector<int> &indices_src,
					const pcl::PointCloud<PointTarget> &cloud_tgt,
					const std::vector<int> &indices_tgt,
					Matrix4 &transformation_matrix) const;

			/** \brief Estimate a rigid rotation transformation between a source and a target point cloud using LM.
			  * \param[in] cloud_src the source point cloud dataset
			  * \param[in] cloud_tgt the target point cloud dataset
			  * \param[in] correspondences the vector of correspondences between source and target point cloud
			  * \param[out] transformation_matrix the resultant transformation matrix
			  */
			inline void
				estimateRigidTransformation(
					const pcl::PointCloud<PointSource> &cloud_src,
					const pcl::PointCloud<PointTarget> &cloud_tgt,
					const pcl::Correspondences &correspondences,
					Matrix4 &transformation_matrix) const;

		protected:
			/** \brief Compute the distance between a source point and its corresponding target point
			  * \param[in] p_src The source point
			  * \param[in] p_tgt The target point
			  * \return The distance between \a p_src and \a p_tgt
			  *
			  * \note Older versions of PCL used this method internally for calculating the
			  * optimization gradient. Since PCL 1.7, a switch has been made to the
			  * computeDistance method using Vector4 types instead. This method is only
			  * kept for API compatibility reasons.
			  */
			virtual MatScalar
				computeDistance(const PointSource &p_src, const PointTarget &p_tgt) const
			{
				Vector4 s(p_src.x, p_src.y, p_src.z, 0);
				Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
				return ((s - t).norm());
			}

			/** \brief Compute the distance between a source point and its corresponding target point
			  * \param[in] p_src The source point
			  * \param[in] p_tgt The target point
			  * \return The distance between \a p_src and \a p_tgt
			  *
			  * \note A different distance function can be defined by creating a subclass of
			  * TransformationEstimationLM and overriding this method.
			  * (See \a TransformationEstimationPointToPlane)
			  */
			virtual MatScalar
				computeDistance(const Vector4 &p_src, const PointTarget &p_tgt) const
			{
				Vector4 t(p_tgt.x, p_tgt.y, p_tgt.z, 0);
				return ((p_src - t).norm());
			}

			/** \brief Temporary pointer to the source dataset. */
			mutable const PointCloudSource *tmp_src_;

			/** \brief Temporary pointer to the target dataset. */
			mutable const PointCloudTarget  *tmp_tgt_;

			/** \brief Temporary pointer to the source dataset indices. */
			mutable const std::vector<int> *tmp_idx_src_;

			/** \brief Temporary pointer to the target dataset indices. */
			mutable const std::vector<int> *tmp_idx_tgt_;

			/** \brief The parameterized function used to warp the source to the target. */
			boost::shared_ptr<pcl::registration::WarpPointRigid<PointSource, PointTarget, MatScalar> > warp_point_;

			/** Base functor all the models that need non linear optimization must
			  * define their own one and implement operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec)
			  * or operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec) dependening on the choosen _Scalar
			  */
			template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
			struct Functor
			{
				typedef _Scalar Scalar;
				enum
				{
					InputsAtCompileTime = NX,
					ValuesAtCompileTime = NY
				};
				typedef Eigen::Matrix<_Scalar, InputsAtCompileTime, 1> InputType;
				typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime, 1> ValueType;
				typedef Eigen::Matrix<_Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

				/** \brief Empty Construtor. */
				Functor() : m_data_points_(ValuesAtCompileTime) {}

				/** \brief Constructor
				  * \param[in] m_data_points number of data points to evaluate.
				  */
				Functor(int m_data_points) : m_data_points_(m_data_points) {}

				/** \brief Destructor. */
				virtual ~Functor() {}

				/** \brief Get the number of values. */
				int
					values() const { return (m_data_points_); }

			protected:
				int m_data_points_;
			};

			struct OptimizationFunctor : public Functor<MatScalar>
			{
				using Functor<MatScalar>::values;

				/** Functor constructor
				  * \param[in] m_data_points the number of data points to evaluate
				  * \param[in,out] estimator pointer to the estimator object
				  */
				OptimizationFunctor(int m_data_points,
					const TransformationEstimationLM_adv *estimator)
					: Functor<MatScalar>(m_data_points), estimator_(estimator)
				{}

				/** Copy constructor
				  * \param[in] src the optimization functor to copy into this
				  */
				inline OptimizationFunctor(const OptimizationFunctor &src) :
					Functor<MatScalar>(src.m_data_points_), estimator_()
				{
					*this = src;
				}

				/** Copy operator
				  * \param[in] src the optimization functor to copy into this
				  */
				inline OptimizationFunctor&
					operator = (const OptimizationFunctor &src)
				{
					Functor<MatScalar>::operator=(src);
					estimator_ = src.estimator_;
					return (*this);
				}

				/** \brief Destructor. */
				virtual ~OptimizationFunctor() {}

				/** Fill fvec from x. For the current state vector x fill the f values
				  * \param[in] x state vector
				  * \param[out] fvec f values vector
				  */
				int
					operator () (const VectorX &x, VectorX &fvec) const;

				const TransformationEstimationLM_adv<PointSource, PointTarget, MatScalar> *estimator_;
			};

			struct OptimizationFunctorWithIndices_adv
            {
                OptimizationFunctorWithIndices_adv(int data_points, const TransformationEstimationLM_adv *estimator)
                        : m_data_points(data_points), estimator_(estimator) {}

                int df(const VectorX& x,  Eigen::Matrix<MatScalar, Eigen::Dynamic, Eigen::Dynamic> &jac) const;
			    int operator()(const VectorX &x, VectorX &fvec);
                int values() const { return m_data_points; }

                const TransformationEstimationLM_adv *estimator_;
                int m_data_points;
            };
			struct OptimizationFunctorWithIndices : public Functor<MatScalar>
			{
				using Functor<MatScalar>::values;

				/** Functor constructor
				  * \param[in] m_data_points the number of data points to evaluate
				  * \param[in,out] estimator pointer to the estimator object
				  */
				OptimizationFunctorWithIndices(int m_data_points,
					const TransformationEstimationLM_adv *estimator)
					: Functor<MatScalar>(m_data_points), estimator_(estimator)
				{}

				/** Copy constructor
				  * \param[in] src the optimization functor to copy into this
				  */
				inline OptimizationFunctorWithIndices(const OptimizationFunctorWithIndices &src)
					: Functor<MatScalar>(src.m_data_points_), estimator_()
				{
					*this = src;
				}

				/** Copy operator
				  * \param[in] src the optimization functor to copy into this
				  */
				inline OptimizationFunctorWithIndices&
					operator = (const OptimizationFunctorWithIndices &src)
				{
					Functor<MatScalar>::operator=(src);
					estimator_ = src.estimator_;
					return (*this);
				}

				/** \brief Destructor. */
				virtual ~OptimizationFunctorWithIndices() {}

				/** Fill fvec from x. For the current state vector x fill the f values
				  * \param[in] x state vector
				  * \param[out] fvec f values vector
				  */
				int
					operator () (const VectorX &x, VectorX &fvec) const;

				const TransformationEstimationLM_adv<PointSource, PointTarget, MatScalar> *estimator_;
			};
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
    }
}

#endif // TRANSFORMATION_ESTIMATE_ADV_H