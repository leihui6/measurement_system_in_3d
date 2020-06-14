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

| library | purpose | link | usage|
| :---:         |     :---:      |         :---: | :---:|
| Eigen 3.3.7   |  core geometric calculation     | [link](http://eigen.tuxfamily.org/index.php?title=Main_Page)    | header-only |
| OpenSceneGraph 3.6.4  |  visulization of point cloud     | [link](http://www.openscenegraph.org/)    | require compile |
| nanoflann 3.6.4  | searching in point cloud     | [link](https://github.com/jlblancoc/nanoflann)   | header-only |
| CGAL 5.0.2  (The Computational Geometry Algorithms Library) | associating with core geometric calculation     | [link](https://www.cgal.org/)   | rquire compile|
| OpenGR(A C++ library for 3D Global Registration)  |  coarsing registration bwtween two arbitrarily located and arbitrarily oriented point clouds.   | [link](https://github.com/STORM-IRIT/OpenGR) or [link](https://storm-irit.github.io/OpenGR/index.html)   | require compile |
| libpointmatcher with `yaml-cpp-pm`   | Iterative Closest Point    | [link](https://github.com/ethz-asl/libpointmatcher)   | require compile |
| cxxopts  |  parsing line command     |[link](https://github.com/jarro2783/cxxopts)  | header-only |

## Demo

![demo](./demo/demo.gif)