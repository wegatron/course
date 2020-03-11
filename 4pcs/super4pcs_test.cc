// #include "gr/io/io.h"
#include "geometry.h"
#include "sampling.h"
#include "match4pcsBase.h"
#include "Functor4pcs.h"
#include "FunctorSuper4pcs.h"
#include "PointPairFilter.h"
#include "shared.h"
#include "logger.h"
#include "demo-utils.h"
#include "demo_io.h"

#include <Eigen/Dense>
#include <boost/regex.hpp>

#include <iostream>
#include <string>
#include <lasser3d_io.h>
#include <zsw_vtk_io.h>
#include "remove_small_island.h"
#include <pcl/filters/radius_outlier_removal.h>

#define sqr(x) ((x) * (x))

using namespace std;
using namespace gr;
using namespace Demo;


// data IO
// IOManager iomananger;


static inline void printS4PCSParameterList()
{
	fprintf(stderr, "\t[ -x (use 4pcs: false by default) ]\n");
	fprintf(stderr, "\t[ --sampled1 (output sampled cloud 1 -- debug+super4pcs only) ]\n");
	fprintf(stderr, "\t[ --sampled2 (output sampled cloud 2 -- debug+super4pcs only) ]\n");
}

struct TransformVisitor
{
	template <typename Derived>
	void operator()(
		float fraction,
		float best_LCP,
		const Eigen::MatrixBase<Derived>& /*transformation*/) const
	{
		if (fraction >= 0)
		{
			printf("done: %d%c best: %f                  \r",
			       static_cast<int>(fraction * 100), '%', best_LCP);
			fflush(stdout);
		}
	}

	constexpr bool needsGlobalTransformation() const { return false; }
};

template <
	typename Matcher,
	typename Options,
	typename Sampler,
	typename TransformVisitor>
Point3D::Scalar computeAlignment(
	const Options& options,
	const Utils::Logger& logger,
	const std::vector<Point3D>& P,
	const std::vector<Point3D>& Q,
	Eigen::Ref<Eigen::Matrix<Point3D::Scalar, 4, 4>> mat,
	const Sampler& sampler,
	TransformVisitor& visitor
)
{
	Matcher matcher(options, logger);
	logger.Log<Utils::Verbose>("Starting registration");
	Point3D::Scalar score = matcher.ComputeTransformation(P, Q, mat, sampler, visitor);


	logger.Log<Utils::Verbose>("Score: ", score);
	logger.Log<Utils::Verbose>("(Homogeneous) Transformation from ",
	                           input2.c_str(),
	                           " to ",
	                           input1.c_str(),
	                           ": \n",
	                           mat);

	if (! outputSampled1.empty())
	{
		logger.Log<Utils::Verbose>("Exporting Sampled cloud 1 to ",
		                           outputSampled1.c_str(),
		                           " ...");
		// iomananger.WriteObject((char *)outputSampled1.c_str(),
		//                        matcher.getFirstSampled(),
		//                        vector<Eigen::Matrix2f>(),
		//                        vector<typename Point3D::VectorType>(),
		//                        vector<tripple>(),
		//                        vector<string>());
		logger.Log<Utils::Verbose>("Export DONE");
	}
	if (! outputSampled2.empty())
	{
		logger.Log<Utils::Verbose>("Exporting Sampled cloud 2 to ",
		                           outputSampled2.c_str(),
		                           " ...");
		// iomananger.WriteObject((char *)outputSampled2.c_str(),
		//                        matcher.getSecondSampled(),
		//                        vector<Eigen::Matrix2f>(),
		//                        vector<typename Point3D::VectorType>(),
		//                        vector<tripple>(),
		//                        vector<string>());
		logger.Log<Utils::Verbose>("Export DONE");
	}

	return score;
}


int main(int argc, char** argv)
{
	using namespace gr;

	vector<Point3D> set1, set2;
	vector<Eigen::Matrix2f> tex_coords1, tex_coords2;
	vector<Point3D::VectorType> normals1, normals2;
	// vector<tripple> tris1, tris2;
	vector<std::string> mtls1, mtls2;

	// Match and return the score (estimated overlap or the LCP).
	Point3D::Scalar score = 0;

	constexpr Utils::LogLevel loglvl = Utils::Verbose;
	using SamplerType = UniformDistSampler;
	using TrVisitorType = std::conditional<loglvl == Utils::NoLog,
	                                       DummyTransformVisitor,
	                                       TransformVisitor>::type;
	using PairFilter = AdaptivePointFilter;

	SamplerType sampler;
	TrVisitorType visitor;
	Utils::Logger logger(loglvl);	

	if (int c = getArgs(argc, argv) != 0)
	{
		printUsage(argc, argv);
		printS4PCSParameterList();
		exit(std::max(c, 0));
	}

	// prepare matcher ressourcesoutputSampled2
	using MatrixType = Eigen::Matrix<Point3D::Scalar, 4, 4>;
	MatrixType mat(MatrixType::Identity());

	// Read the inputs.
	// "F:/data/3d-data/map_cuixian/Node3DDir-20181110T171800/LASER3D_0_6_6986.72.bin"
	std::cout << input1 << std::endl;
	// load point cloud and remove small island

	load_laser3d_frame(input1, set1);
	std::stringstream ss;
	ss << std::setw(4) << std::setfill('0') << cur_id() << ".vtk";
	remove_small_island(set1, 0.4, 200, "f:/data/tmp/4pcs_res/p_filtered_"+ss.str());

	//  read map, and extract the local map
	std::cout << input2 << std::endl;

	load_local_map(input2, cur_pos(), set2);
	write_vtk("f:/data/tmp/4pcs_res/q_filtered_"+ss.str(), set2);
	try
	{
		if (use_super4pcs)
		{
			using MatcherType = Match4pcsBase<FunctorSuper4PCS, TrVisitorType, AdaptivePointFilter, AdaptivePointFilter
			                                  ::Options>;
			using OptionType = MatcherType::OptionsType;

			OptionType options;
			if (! setOptionsFromArgs(options, logger))
			{
				exit(-2); /// \FIXME use status codes for error reporting
			}

			score = computeAlignment<MatcherType>(options, logger, set1, set2, mat, sampler, visitor);
		}
		else
		{
			using MatcherType = Match4pcsBase<Functor4PCS, TrVisitorType, AdaptivePointFilter, AdaptivePointFilter::
			                                  Options>;
			using OptionType = MatcherType::OptionsType;

			OptionType options;
			if (! setOptionsFromArgs(options, logger))
			{
				exit(-2); /// \FIXME use status codes for error reporting
			}

			score = computeAlignment<MatcherType>(options, logger, set1, set2, mat, sampler, visitor);
		}
	}
	catch (const std::exception& e)
	{
		logger.Log<Utils::ErrorReport>("[Error]: ", e.what());
		logger.Log<Utils::ErrorReport>("Aborting with code -3 ...");
		return -3;
	}
	catch (...)
	{
		logger.Log<Utils::ErrorReport>("[Unknown Error]: Aborting with code -4 ...");
		return -4;
	}

	if (! output.empty())
	{
		logger.Log<Utils::Verbose>("Exporting Registered geometry to ",
		                           output.c_str(),
		                           "...");
		//MatrixType mat_inv = mat.inverse();
		Utils::TransformPointCloud(set2, mat);
		write_vtk2(output, set1, set2);
		// hosdorff distance
		//output_res(set1, set2);
		logger.Log<Utils::Verbose>("Export DONE");
	}
	return 0;
}
