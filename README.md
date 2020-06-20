# Measurement System in 3D

This is my final project in master's degree,  that is *Automatic Industrial Measurement System Based on Registration*.

## Steps

The work flow of this system was shown as following. All steps will be implemented in C++ and run on console(Windows or Linux) considering this is a huge project and I don't have too much time:

- [x] [Labeling a set of points representing the shape to be detected](./labeling_points)
- [x] [Rough registration](./rough_registration)
- [x] [Fine registration](./fine_registration)
- [ ] Searching the points to be measured in specified point cloud
- [ ] [delay]Combining all components
- [ ] [delay]Making it across-platform including `Windows (windows 10)` and `Linux(Ubuntu *)`

please use this tool ([click](./display_fine_and_coarse))) to visualize coarse and fine registration.

## Compiler

For now, all steps are testing and running on `Windows 10` with `visual studio 2017`.

## Requirements

Every step should has its dependencies libraries, also might require the whole libs as follows.

| Library | Purpose | Official Website | Usage| Note |
| :---         |     :---      | :--- | :---|:---|
 | Eigen 3.3.7   |  Core geometric calculation     | [link](http://eigen.tuxfamily.org/index.php?title=Main_Page)    | header-only | |
| OpenSceneGraph 3.6.4  |  Visulization of point cloud     | [link](http://www.openscenegraph.org/)    | require compile | |
| nanoflann 3.6.4  | Visiting operation in point cloud     | [link](https://github.com/jlblancoc/nanoflann)   | header-only | |
| CGAL 5.0.2  (The Computational Geometry Algorithms Library) | Associating with core geometric calculation     | [link](https://www.cgal.org/)   | rquire compile| |
| OpenGR(A C++ library for 3D Global Registration)  |  Implement Super4PCS for coarse registration  | [link](https://github.com/STORM-IRIT/OpenGR) or [link](https://storm-irit.github.io/OpenGR/index.html)   | header-only | |
| libpointmatcher with its `yaml-cpp-pm`   | Implement  ICP  for fine registration    | [link](https://github.com/ethz-asl/libpointmatcher)   | require compile | [modified version](https://github.com/Gltina/libpointmatcher) that support export matrix per iteration instead of completed vtk-file when setting `dumpReading`|

## Demo

![demo](./demo/demo.gif)