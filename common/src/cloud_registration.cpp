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

void cloud_registration::coarse_registration(std::vector<point_3d>& points1, std::vector<point_3d>& points2, Eigen::Matrix4f & ret_mat)
{
	using MatcherType = gr::Match4pcsBase<gr::FunctorSuper4PCS, gr::Point3D<float>, gr::DummyTransformVisitor, gr::AdaptivePointFilter, gr::AdaptivePointFilter::Options>;

	std::vector<gr::Point3D<float> > set1, set2;

	convert_to_openGR_points(points1, set1);

	convert_to_openGR_points(points2, set2);

	std::vector<Eigen::Matrix2f> tex_coords1, tex_coords2;

	//std::vector<typename gr::Point3D<float>::VectorType> normals1, normals2;

	std::vector<std::string> mtls1, mtls2;

	// dummy calls, to test symbols accessibility
	// check availability of the Utils functions
	//gr::Utils::CleanInvalidNormals(set1, normals1);

	// Our matcher.
	MatcherType::OptionsType options;

	// Set parameters.
	//typename MatcherType::MatrixType mat;

	double overlap(1);

	options.configureOverlap(overlap);

	typename gr::Point3D<float>::Scalar score = 0;

	constexpr gr::Utils::LogLevel loglvl = gr::Utils::Verbose;

	gr::Utils::Logger logger(loglvl);

	gr::UniformDistSampler<gr::Point3D<float> > sampler;

	gr::DummyTransformVisitor visitor;

	MatcherType matcher(options, logger);

	score = matcher.ComputeTransformation(set1, set2, ret_mat, sampler, visitor);

	//std::cout << mat << std::endl;

	logger.Log<gr::Utils::Verbose>("Score: ", score);

	this->coarse_transform_matrix = ret_mat;
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

	if (ifs.is_open()) ifs.close();
	
	PM::TransformationParameters T = icp(reading_points_icp, reference_points_icp);

	ret_mat = T;
}

void cloud_registration::get_final_transform_matrix(Eigen::Matrix4f & final_transform_matrix)
{
	final_transform_matrix = fine_transform_matrix * coarse_transform_matrix;
}
