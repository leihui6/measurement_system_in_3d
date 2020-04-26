#include "cloud_point.h"

point_3d::point_3d()
	:x(0), y(0), z(0),
	nx(0), ny(0), nz(0),
	r(0), g(0), b(0)
{
}

void point_3d::set_xyz(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void point_3d::set_nxyz(float nx, float ny, float nz)
{
	this->nx = nx;
	this->ny = ny;
	this->nz = nz;
}

void point_3d::set_rgb(float r, float g, float b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

void point_3d::to_eigen_vector4f(Eigen::Vector4f & vector4f_p)
{
	vector4f_p(0, 0) = x;
	vector4f_p(1, 0) = y;
	vector4f_p(2, 0) = z;
	vector4f_p(3, 0) = 1;
}

void point_3d::do_transform(Eigen::Matrix4f & t, point_3d & p)
{
	Eigen::Vector4f tmp(x, y, z, 1);

	tmp = t * tmp;

	p.set_xyz(tmp(0, 0), tmp(1, 0), tmp(2, 0));
}

void convert_to_CGAL_points(std::vector<point_3d>& points, std::vector<CGAL::Simple_cartesian<float>::Point_3> & cgal_points)
{
	//cgal_points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		cgal_points.push_back(CGAL::Simple_cartesian<float>::Point_3(points[i].x, points[i].y, points[i].z));
	}
}

void convert_to_original_points(std::vector<CGAL::Simple_cartesian<float>::Point_3>& cgal_points, std::vector<point_3d>& points)
{
	points.resize(cgal_points.size());

	for (size_t i = 0; i < cgal_points.size(); ++i)
	{
		points[i].set_xyz(cgal_points[i].x(), cgal_points[i].y(), cgal_points[i].z());
	}
}

void convert_to_openGR_points(std::vector<point_3d>& points, std::vector<gr::Point3D<float>>& opengr_points)
{
	for (size_t i = 0; i < points.size(); ++i)
	{
		opengr_points.push_back(gr::Point3D<float>(points[i].x, points[i].y, points[i].z));
	}
}

void convert_to_pointMatcher_points(std::vector<point_3d>& points, PointMatcher<float>::DataPoints & DP)
{
	const PointMatcherIO<float>::SupportedLabels & externalLabels = PointMatcherIO<float>::getSupportedExternalLabels();

	PointMatcherIO<float>::LabelGenerator featLabelGen, descLabelGen, timeLabelGen;

	featLabelGen.add(externalLabels[0].internalName);
	featLabelGen.add(externalLabels[1].internalName);
	featLabelGen.add(externalLabels[2].internalName);
	featLabelGen.add(externalLabels[3].internalName);

	//descLabelGen.add(externalLabels[4].internalName);
	//descLabelGen.add(externalLabels[5].internalName);
	//descLabelGen.add(externalLabels[6].internalName);

	PointMatcher<float>::Matrix features = PointMatcher<float>::Matrix(4, points.size());

	//PointMatcher<float>::Matrix describe = PointMatcher<float>::Matrix(3, points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		features(0, i) = points[i].x;
		features(1, i) = points[i].y;
		features(2, i) = points[i].z;
		features(3, i) = 1;

		//describe(0, i) = points[i].nx;
		//describe(1, i) = points[i].ny;
		//describe(2, i) = points[i].nz;
	}

	DP.features = features;
	DP.featureLabels = featLabelGen.getLabels();

	//DP.features = describe;
	//DP.featureLabels = descLabelGen.getLabels();

	if (!DP.featureExists("pad"))
	{
		DP.addFeature("pad", PointMatcher<float>::Matrix::Ones(1, features.cols()));
	}

	std::cout << std::endl;
}

void transform_points(std::vector<point_3d>& points, Eigen::Matrix4f & t, std::vector<point_3d>& ret_points)
{
	ret_points.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		points[i].do_transform(t, ret_points[i]);
	}
}

void save_points(std::vector<point_3d>& points, const std::string & filename)
{
	std::ofstream of(filename, std::ios::out);

	if (filename.find(".txt") != std::string::npos)
	{
		for (size_t i = 0; i < points.size(); ++i)
		{
			of
				<< points[i].x << " " << points[i].y << " " << points[i].z << " "

				<< points[i].nx << " " << points[i].ny << " " << points[i].nz << " "

				<< points[i].r << " " << points[i].g << " " << points[i].b

				<< std::endl;
		}

		of.close();
	}
}
