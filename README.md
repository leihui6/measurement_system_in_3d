# Measurement System in 3D

This is my final project in master's degree,  that is *Automatic Industrial Measurement System Based on Registration*.

## Steps

The work flow of this system was shown as following. All steps will be implemented in C++ and run on console(Windows or Linux) considering this is a huge project and I don't have too much time:

- [ ] [Labeling a set of points representing the shape to be detected](./labeling_points)
- [ ] Rough registration
- [ ] Fine registration
- [ ] Searching the points to be measured in specified point cloud
- [ ] Make it across-platform including `Windows (windows 10)` and `Linux(Ubuntu *)`

## Compiler

For now, all steps are testing and running on `Windows 10` with `visual studio 2017`.

## Whole requirements

Every step should has its dependencies libraries, also might require the whole libs as follows.

- Eigen 3.3.7 [link](http://eigen.tuxfamily.org/index.php?title=Main_Page)
  - for core geometric calculation
- OpenSceneGraph 3.6.4 [link](http://www.openscenegraph.org/)
  - for visulization of point cloud
- nanoflann [link](https://github.com/jlblancoc/nanoflann)
  - for searching in point cloud
- CGAL 5.0.2 (The Computational Geometry Algorithms Library) [link](https://www.cgal.org/)
  - for associating with core geometric calculation
- OpenGR(A C++ library for 3D Global Registration) [link](https://github.com/STORM-IRIT/OpenGR)
  - for coarsing registration bwtween two arbitrarily located and arbitrarily oriented point clouds.
- libpointmatcher with `yaml-cpp-pm` [link](https://github.com/ethz-asl/libpointmatcher) 
  - for "Iterative Closest Point"
- cxxopts [link](https://github.com/jarro2783/cxxopts)
  - for parsing line command

## Demo

![demo](./demo/demo.gif)