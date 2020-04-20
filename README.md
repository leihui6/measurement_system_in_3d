# Measurement System in 3D
This is my final project in master's degree

# Requirements
- Eigen 3.3.7 [link](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - for core geometric calculation
- OpenSceneGraph 3.6.4 [link](http://www.openscenegraph.org/)
  - for visulization of point cloud
- nanoflann [link](https://github.com/jlblancoc/nanoflann)
  - for searching in point cloud
- CGAL(The Computational Geometry Algorithms Library) [link](https://www.cgal.org/)
  - for associating with core geometric calculation
- OpenGR(A C++ library for 3D Global Registration) [link](https://github.com/STORM-IRIT/OpenGR)
  - for coarsing registration bwtween two arbitrarily located and arbitrarily oriented point clouds.
- libpointmatcher with `yaml-cpp-pm` [link](https://github.com/ethz-asl/libpointmatcher) 
  - for "Iterative Closest Point"

# Demo
![](./demo/demo.gif)