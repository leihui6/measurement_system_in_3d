#include "cloud_registration.h"

#include <gr/algorithms/FunctorSuper4pcs.h>
#include <gr/utils/geometry.h>
#include <gr/accelerators/utils.h>
#include <gr/algorithms/PointPairFilter.h>

cloud_registration::cloud_registration()
{
}


cloud_registration::~cloud_registration()
{
}

void cloud_registration::coarse_registration(std::vector<point_3d>& readning_point_cloud, std::vector<point_3d>& reference_point_cloud, Eigen::Matrix4f & ret_mat)
{
	using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, TrVisitorType, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;

	std::vector<gr::Point3D<float> > reading_point_cloud_gr, reference_point_cloud_gr;

	convert_to_openGR_points(readning_point_cloud, reading_point_cloud_gr);

	convert_to_openGR_points(reference_point_cloud, reference_point_cloud_gr);

	// Our matcher.
	MatcherType::OptionsType options;

	constexpr gr::Utils::LogLevel loglvl = gr::Utils::Verbose;

	gr::Utils::Logger logger(loglvl);

	MatcherType matcher(options, logger);

	gr::UniformDistSampler<gr::Point3D<float> > sampler;

	TrVisitorType visitor;

	options.configureOverlap(double(1.0));

	std::cout
		<< "getOverlapEstimation:" << options.getOverlapEstimation() << "\n"
		// delta, used to compute the LCP between the two models
		<< "options.delta:" << options.delta << "\n"
		// number of samples used for the matching
		<< "options.sample_size:" << options.sample_size << "\n"
		// default : -1
		<< "options.max_normal_difference:" << options.max_normal_difference << "\n"
		// default : -1
		<< "options.max_color_distance:" << options.max_color_distance << "\n"
		// maximum computation time in seconds
		<< "options.max_time_seconds:" << options.max_time_seconds << "\n";

	typename gr::Point3D<float>::Scalar score = 0;

	score = matcher.ComputeTransformation(reference_point_cloud_gr, reading_point_cloud_gr, ret_mat, sampler, visitor);

	logger.Log<gr::Utils::Verbose>("Score: ", score);

	//ret_mat.transposeInPlace();
}

void cloud_registration::fine_registration(std::vector<point_3d>& reading_points, std::vector<point_3d>& reference_points, const std::string & configuration_file_name, Eigen::Matrix4f &ret_mat)
{
	typedef PointMatcher<float> PM;

	PM::DataPoints reading_points_icp, reference_points_icp;

	convert_to_pointMatcher_points(reading_points, reading_points_icp);

	convert_to_pointMatcher_points(reference_points, reference_points_icp);

	PM::ICP icp;

	std::ifstream ifs(configuration_file_name);

	icp.loadFromYaml(ifs);

	PM::TransformationParameters T = icp(reading_points_icp, reference_points_icp);

	if (ifs.is_open()) ifs.close();

	ret_mat = T;
}

void cloud_registration::get_final_transform_matrix(Eigen::Matrix4f & final_transform_matrix)
{
	final_transform_matrix = fine_transform_matrix * coarse_transform_matrix;
}
