#include "cloud_fitting.h"

cloud_fitting::cloud_fitting()
	:m_num_theta_samples(1024),
	m_num_phi_samples(512)
{
	m_num_threads = std::thread::hardware_concurrency();
}


cloud_fitting::~cloud_fitting()
{

}

void cloud_fitting::fitting_line_3d_linear_least_squares(std::vector<point_3d>& points, line_func_3d & line_func)
{
	if (points.size() < 2) return;

	Eigen::MatrixXf m;
	m.resize(points.size(), 3);
	point_3d mean_p;
	centroid_from_points(points, mean_p);

	for (size_t i = 0; i < points.size(); i++)
	{
		m(i, 0) = points[i].x - mean_p.x;
		m(i, 1) = points[i].y - mean_p.y;
		m(i, 2) = points[i].z - mean_p.z;
	}

	//std::cout << m << std::endl;

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::MatrixXf rm = svd.matrixV();

	if (rm.cols() > 1)
	{
		line_func.set_xyz(mean_p.x, mean_p.y, mean_p.z);
		line_func.set_nml(rm(0, 0), rm(1, 0), rm(2, 0));
	}
}

void cloud_fitting::fitting_plane_3d_linear_least_squares(std::vector<point_3d>& points, plane_func_3d & plane_func)
{
	if (points.size() < 3) return;

	Eigen::MatrixXf m;
	m.resize(points.size(), 3);
	point_3d mean_p;
	centroid_from_points(points, mean_p);

	for (size_t i = 0; i < points.size(); i++)
	{
		m(i, 0) = points[i].x - mean_p.x;
		m(i, 1) = points[i].y - mean_p.y;
		m(i, 2) = points[i].z - mean_p.z;
	}

	//std::cout << m << std::endl;

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::MatrixXf rm = svd.matrixV();

	if (rm.cols() == 3)
	{
		plane_func.set_abcd(rm(0, 2), rm(1, 2), rm(2, 2), mean_p);
	}
}

float cloud_fitting::fitting_cylinder_linear_least_squares(std::vector<point_3d>& points, cylinder_func & _cylinder_func)
{
	m_X.clear();
	float mInvNumPoints = (float)0;
	_cylinder_func.axis.set_xyz(0, 0, 0);// = Vector3::Zero();
	_cylinder_func.axis.set_nml(0, 0, 0);// = Vector3::Zero();
	_cylinder_func.radius = 0.0f;
	_cylinder_func.height = 0.0f;

	// Validate the input parameters.
	if (points.size() < 6)
	{
		return std::numeric_limits<float>::max();
	}

	Eigen::Vector3f average;
	preprocess(points, average);

	// Fit the points based on which constructor the caller used.  The
	// direction is either estimated or selected directly or
	// indirectly by the caller.  The center and squared radius are
	// estimated.
	Eigen::Vector3f minPC, minW;
	float minRSqr, minError;

	// Search the hemisphere for the vector that leads to minimum
	// error and use it for the cylinder axis.
	//minError = computeSingleThreaded(minPC, minW, minRSqr);

	minError = computeMultiThreaded(minPC, minW, minRSqr);

	// Translate back to the original space by the average of the
	// points.
	_cylinder_func.axis.origin = (minPC + average);
	_cylinder_func.axis.direction = minW;

	// Compute the cylinder radius.
	_cylinder_func.radius = std::sqrt(minRSqr);

	// Project the points onto the cylinder axis and choose the
	// cylinder center and cylinder height as described in the
	// comments at the top of this header file.
	float tmin = 0.0f, tmax = 0.0f;
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		float t = _cylinder_func.axis.direction.dot(points[i].get_vector3f() - _cylinder_func.axis.origin);
		tmin = std::min(t, tmin);
		tmax = std::max(t, tmax);
	}

	_cylinder_func.axis.origin += ((tmin + tmax) * 0.5f) * _cylinder_func.axis.direction;
	_cylinder_func.height = tmax - tmin;

	return minError;
}

void cloud_fitting::preprocess(std::vector<point_3d>& points, Eigen::Vector3f & average)
{
	size_t num_points = points.size();
	m_X.resize(num_points);
	float mInvNumPoints = 1.0f / (float)num_points;

	// Copy the points and translate by the average for numerical
	// robustness.
	average.setZero();
	for (unsigned int i = 0; i < num_points; ++i)
	{
		average += points[i].get_vector3f();
	}
	average *= mInvNumPoints;

	for (unsigned int i = 0; i < num_points; ++i)
	{
		m_X[i] = points[i].get_vector3f() - average;
	}
	//Vector<6, Real> zero{ (Real)0 };
	//std::vector<Vector<6, Real>> products(mX.size(), zero);
	Eigen::Array<float, 6, 1>zero{ 0 };
	std::vector<Eigen::Array<float, 6, 1>> products(m_X.size(), zero);
	//Vector<6, Real> mMu;
	m_Mu = zero;
	for (size_t i = 0; i < m_X.size(); ++i)
	{
		products[i][0] = m_X[i][0] * m_X[i][0];
		products[i][1] = m_X[i][0] * m_X[i][1];
		products[i][2] = m_X[i][0] * m_X[i][2];
		products[i][3] = m_X[i][1] * m_X[i][1];
		products[i][4] = m_X[i][1] * m_X[i][2];
		products[i][5] = m_X[i][2] * m_X[i][2];
		m_Mu[0] += products[i][0];
		m_Mu[1] += 2.0f * products[i][1];
		m_Mu[2] += 2.0f * products[i][2];
		m_Mu[3] += products[i][3];
		m_Mu[4] += 2.0f * products[i][4];
		m_Mu[5] += products[i][5];
	}
	m_Mu *= mInvNumPoints;

	m_F0.setZero();
	m_F1.setZero();
	m_F2.setZero();
	for (size_t i = 0; i < m_X.size(); ++i)
	{
		Eigen::Matrix<float, 6, 1> delta;
		delta[0] = products[i][0] - m_Mu[0];
		delta[1] = 2.0f * products[i][1] - m_Mu[1];
		delta[2] = 2.0f * products[i][2] - m_Mu[2];
		delta[3] = products[i][3] - m_Mu[3];
		delta[4] = 2.0f * products[i][4] - m_Mu[4];
		delta[5] = products[i][5] - m_Mu[5];
		m_F0(0, 0) += products[i][0];
		m_F0(0, 1) += products[i][1];
		m_F0(0, 2) += products[i][2];
		m_F0(1, 1) += products[i][3];
		m_F0(1, 2) += products[i][4];
		m_F0(2, 2) += products[i][5];
		//mF1 += OuterProduct(mX[i], delta);
		//mF2 += OuterProduct(delta, delta);
		m_F1 += m_X[i] * delta.transpose();
		m_F2 += delta * delta.transpose();
	}
	m_F0 *= mInvNumPoints;
	m_F0(1, 0) = m_F0(0, 1);
	m_F0(2, 0) = m_F0(0, 2);
	m_F0(2, 1) = m_F0(1, 2);
	m_F1 *= mInvNumPoints;
	m_F2 *= mInvNumPoints;
}

float cloud_fitting::computeSingleThreaded(Eigen::Vector3f & minPC, Eigen::Vector3f & minW, float & minRSqr)
{
	float const iMultiplier = (float)TWO_PI /  m_num_theta_samples;
	float const jMultiplier = (float)HALF_PI / m_num_phi_samples;

	// Handle the north pole (0,0,1) separately.
	minW = { 0.0f, 0.0f, 1.0f };
	float minError = G(minW, minPC, minRSqr);

	for (unsigned int j = 1; j <= m_num_phi_samples; ++j)
	{
		float phi = jMultiplier * static_cast<float>(j);  // in [0,pi/2]
		float csphi = std::cos(phi);
		float snphi = std::sin(phi);
		for (unsigned int i = 0; i < m_num_theta_samples; ++i)
		{
			float theta = iMultiplier * static_cast<float>(i);  // in [0,2*pi)
			float cstheta = std::cos(theta);
			float sntheta = std::sin(theta);
			Eigen::Vector3f W{ cstheta * snphi, sntheta * snphi, csphi };
			Eigen::Vector3f PC;
			float rsqr;
			float error = G(W, PC, rsqr);
			if (error < minError)
			{
				minError = error;
				minRSqr = rsqr;
				minW = W;
				minPC = PC;
			}
		}
	}
	return minError;
}

float cloud_fitting::computeMultiThreaded(Eigen::Vector3f & minPC, Eigen::Vector3f & minW, float & minRSqr)
{
	float const iMultiplier = TWO_PI / (float)m_num_theta_samples;
	float const jMultiplier = HALF_PI / (float)m_num_phi_samples;

	// Handle the north pole (0,0,1) separately.
	minW = { 0.0f, 0.0f, 1.0f };
	float minError = G(minW, minPC, minRSqr);

	struct Local
	{
		float error;
		float rsqr;
		Eigen::Vector3f W;
		Eigen::Vector3f PC;
		unsigned int jmin;
		unsigned int jmax;
	};

	std::vector<Local> local(m_num_threads);
	unsigned int numPhiSamplesPerThread = m_num_phi_samples / m_num_threads;

	for (unsigned int t = 0; t < m_num_threads; ++t)
	{
		local[t].error = std::numeric_limits<float>::max();
		local[t].rsqr = 0.0f;
		local[t].W = Eigen::Vector3f::Zero();
		local[t].PC = Eigen::Vector3f::Zero();
		local[t].jmin = numPhiSamplesPerThread * t;
		local[t].jmax = numPhiSamplesPerThread * (t + 1);
	}
	local[m_num_threads - 1].jmax = m_num_phi_samples + 1;

	std::vector<std::thread> process(m_num_threads);
	for (unsigned int t = 0; t < m_num_threads; ++t)
	{
		process[t] = std::thread
		(
			[this,t, iMultiplier, jMultiplier, &local]()
		{
			for (unsigned int j = local[t].jmin; j < local[t].jmax; ++j)
			{
				// phi in [0,pi/2]
				float phi = jMultiplier * static_cast<float>(j);
				float csphi = std::cos(phi);
				float snphi = std::sin(phi);
				for (unsigned int i = 0; i < m_num_theta_samples; ++i)
				{
					// theta in [0,2*pi)
					float theta = iMultiplier * static_cast<float>(i);
					float cstheta = std::cos(theta);
					float sntheta = std::sin(theta);
					Eigen::Vector3f  W{ cstheta * snphi, sntheta * snphi, csphi };
					Eigen::Vector3f  PC;
					float rsqr;
					float error = G(W, PC, rsqr);
					if (error < local[t].error)
					{
						local[t].error = error;
						local[t].rsqr = rsqr;
						local[t].W = W;
						local[t].PC = PC;
					}
				}
			}
		}
		);
	}

	for (unsigned int t = 0; t < m_num_threads; ++t)
	{
		process[t].join();

		if (local[t].error < minError)
		{
			minError = local[t].error;
			minRSqr = local[t].rsqr;
			minW = local[t].W;
			minPC = local[t].PC;
		}
	}

	return minError;
}

float cloud_fitting::G(Eigen::Vector3f const & W, Eigen::Vector3f & PC, float & rsqr)
{
	typedef Eigen::Matrix3f Matrix3x3;
	Matrix3x3 P = Matrix3x3::Identity() - (W* W.transpose());
	Matrix3x3 S;
	S <<
		0, -W[2], W[1],
		W[2], 0, -W[0],
		-W[1], W[0], 0;

	Matrix3x3 A = P * m_F0 * P;
	Matrix3x3 hatA = -(S * A * S);
	Matrix3x3 hatAA = hatA * A;
	float trace = hatAA.trace();
	Matrix3x3 Q = hatA / trace;
	Eigen::Matrix<float, 6, 1> pVec;
	pVec
		<< P(0, 0), P(0, 1), P(0, 2), P(1, 1), P(1, 2), P(2, 2);

	Eigen::Vector3f alpha = m_F1 * pVec;
	Eigen::Vector3f beta = Q * alpha;
	float G = (pVec.dot(m_F2 * pVec) - 4.0f * alpha.dot(beta) + 4.0f * beta.dot(m_F0 * beta)) / (float)m_X.size();

	PC = beta;
	rsqr = pVec.transpose().dot(m_Mu) + PC.dot(PC);
	return G;
}

//void cloud_fitting::fitting_cylinder_ransac(std::vector<point_3d>& points, cylinder_func & _cylinder_func, size_t iteration_count)
//{
	//_cylinder_func.r = 0.0;

	//std::vector<size_t> random_vec(points.size());

	//for (size_t i = 0; i < random_vec.size(); ++i)
	//{
	//	random_vec[i] = i;
	//}

	//int i = 0;

	//float min_deviation = FLT_MAX, required_probability = 0.8;

	////float cylinder_r = 0;

	////point_3d cylinder_line_point, cylinder_line_direction;

	//while (true)
	//{
	//	if (i++ > iteration_count)
	//	{
	//		break;
	//	}

	//	std::random_shuffle(random_vec.begin(), random_vec.end());

	//	//std::cout << random_vec[0] << " " << random_vec[1] << " " << random_vec[2] << std::endl;

	//	plane_func_3d plane_func;

	//	plane_function_from_three_points(points[random_vec[0]], points[random_vec[1]], points[random_vec[2]], plane_func);

	//	std::vector<point_3d> use_for_cylinder;

	//	points_on_plane(points, use_for_cylinder, plane_func, 0.2);

	//	point_3d centriod_point;

	//	centroid_from_points(use_for_cylinder, centriod_point);

	//	float mean_distance = 0.0;

	//	mean_distance_from_point_to_points(use_for_cylinder, centriod_point, mean_distance);

	//	//std::cout << "mean_distance:" << mean_distance << std::endl;

	//	line_func_3d line_func;

	//	line_func.set_xyz(centriod_point.x, centriod_point.y, centriod_point.z);

	//	line_func.set_nml(plane_func.a, plane_func.b, plane_func.c);

	//	std::vector<float> distance_to_cylinder_line;

	//	distance_points_to_line(use_for_cylinder, line_func, distance_to_cylinder_line);

	//	float deviation = 0.0;// , probability = 0.0;

	//	//probability_close_to_value(distance_to_cylinder_line, mean_distance, 0.1, probability);

	//	standard_deviation(distance_to_cylinder_line, deviation);

	//	if (deviation < min_deviation)
	//	//if (probability > required_probability)
	//	{
	//		min_deviation = deviation;

	//		_cylinder_func.r = mean_distance;

	//		_cylinder_func.m_line_func.set_xyz(centriod_point.x, centriod_point.y, centriod_point.z);

	//		_cylinder_func.m_line_func.set_nml(plane_func.a, plane_func.b, plane_func.c);

	//		//required_probability = probability;
	//	}
	//	else
	//	{
	//		std::cout
	//			<< "deviation=" << deviation << "\n";
	//	}
	//}
	//std::cout
	//	<< "min_deviation=" << min_deviation << "\n"
	//	<< "cylinder_r=" << _cylinder_func.r << "\n";
//}

//void cloud_fitting::get_drawable_cylinder_using_bottom_plane(std::vector<point_3d>& points, point_3d & bottom_center_p, plane_func_3d & plane_func, point_3d & center_p, float & radius, float & height)
//{
//
//}

/*
void cloud_fitting::get_convex_hull_from_points(std::vector<point_3d>& points, std::vector<point_3d>& convex_hull_points)
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel  EK;

	std::vector<EK::Point_3> cgal_points;

	for (size_t i = 0; i < points.size(); ++i)
	{
		EK::Point_3 p(points[i].x, points[i].y, points[i].z);

		cgal_points.push_back(p);
	}

	CGAL::Polyhedron_3<EK> poly;

	CGAL::convex_hull_3(cgal_points.begin(), cgal_points.end(), poly);

	convex_hull_points.resize(poly.size_of_vertices());

	size_t i = 0;
	for (CGAL::Polyhedron_3<CGAL::Exact_predicates_inexact_constructions_kernel>::Vertex_iterator it = poly.vertices_begin();
		it != poly.vertices_end();
		++it)
	{
		convex_hull_points[i].set_xyz(
			it->point()[0],
			it->point()[1],
			it->point()[2]
		);
		++i;
	}
}
*/
