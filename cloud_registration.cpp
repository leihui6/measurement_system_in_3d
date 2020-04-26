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

void cloud_registration::fine_registration(std::vector<point_3d>& points1, std::vector<point_3d>& points2, Eigen::Matrix4f &ret_mat)
{
	typedef PointMatcher<float> PM;
	//typedef PM::DataPoints DP;

	PointMatcher<float>::DataPoints data_points_1, data_points_2;

	convert_to_pointMatcher_points(points1, data_points_1);

	convert_to_pointMatcher_points(points2, data_points_2);

	// Create the default ICP algorithm
	PointMatcher<float>::ICP icp;

	// See the implementation of setDefault() to create a custom ICP algorithm
	icp.setDefault();
	/*
	PointMatcherSupport::Parametrizable::Parameters params;
	std::string name;

	// Uncomment for console outputs
	setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

	// Prepare reading filters
	name = "MinDistDataPointsFilter";
	params["minDist"] = "5.0";
	std::shared_ptr<PM::DataPointsFilter> minDist_read =
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	//name = "RandomSamplingDataPointsFilter";
	//params["prob"] = "0.2";
	//std::shared_ptr<PM::DataPointsFilter> rand_read =
	//	PM::get().DataPointsFilterRegistrar.create(name, params);
	//params.clear();

	//name = "RandomSamplingDataPointsFilter";
	//params["prob"] = "0.2";
	//std::shared_ptr<PM::DataPointsFilter> rand_ref =
	//	PM::get().DataPointsFilterRegistrar.create(name, params);
	//params.clear();

	// Prepare reference filters
	name = "MinDistDataPointsFilter";
	params["minDist"] = "5.0";
	std::shared_ptr<PM::DataPointsFilter> minDist_ref =
		PM::get().DataPointsFilterRegistrar.create(name, params);
	params.clear();

	// Prepare matching function
	name = "KDTreeMatcher";
	params["knn"] = "20";
	params["epsilon"] = "3.16";
	std::shared_ptr<PM::Matcher> kdtree =
		PM::get().MatcherRegistrar.create(name, params);
	params.clear();

	// Prepare outlier filters
	name = "TrimmedDistOutlierFilter";
	params["ratio"] = "0.75";
	std::shared_ptr<PM::OutlierFilter> trim =
		PM::get().OutlierFilterRegistrar.create(name, params);
	params.clear();

	// Prepare error minimization
	name = "PointToPointErrorMinimizer";
	std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
		PM::get().ErrorMinimizerRegistrar.create(name);

	// Prepare transformation checker filters
	name = "CounterTransformationChecker";
	params["maxIterationCount"] = "150";
	std::shared_ptr<PM::TransformationChecker> maxIter =
		PM::get().TransformationCheckerRegistrar.create(name, params);
	params.clear();

	name = "DifferentialTransformationChecker";
	params["minDiffRotErr"] = "0.001";
	params["minDiffTransErr"] = "0.01";
	params["smoothLength"] = "4";
	std::shared_ptr<PM::TransformationChecker> diff =
		PM::get().TransformationCheckerRegistrar.create(name, params);
	params.clear();

	// Prepare inspector
	std::shared_ptr<PM::Inspector> nullInspect =
		PM::get().InspectorRegistrar.create("NullInspector");
	params.clear();

	// Prepare transformation
	std::shared_ptr<PM::Transformation> rigidTrans =
		PM::get().TransformationRegistrar.create("RigidTransformation");

	// Build ICP solution
	icp.readingDataPointsFilters.push_back(minDist_read);
	//icp.readingDataPointsFilters.push_back(rand_read);

	icp.referenceDataPointsFilters.push_back(minDist_ref);
	//icp.referenceDataPointsFilters.push_back(rand_ref);

	icp.matcher = kdtree;

	icp.outlierFilters.push_back(trim);

	icp.errorMinimizer = pointToPoint;

	icp.transformationCheckers.push_back(maxIter);
	icp.transformationCheckers.push_back(diff);

	// toggle to write vtk files per iteration
	icp.inspector = nullInspect;
	//icp.inspector = vtkInspect;

	icp.transformations.push_back(rigidTrans);
	*/
	// Compute the transformation to express data in ref
	PointMatcher<float>::TransformationParameters T = icp(data_points_2, data_points_1);

	//PM::DataPoints data_out(data_points_2);

	//icp.transformations.apply(data_out, T);

	ret_mat = T;

	this->fine_transform_matrix = T;
}

void cloud_registration::get_final_transform_matrix(Eigen::Matrix4f & final_transform_matrix)
{
	final_transform_matrix = fine_transform_matrix * coarse_transform_matrix;
}
