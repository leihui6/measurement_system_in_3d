# Measurement System in 3D

This is my final project of master's degree, that is, *Automatic Industrial Measurement System Based on Registration*.

## Steps

The work flow of this system was shown as follows.

- [x] [Labeling/Marking a set of points representing the shape to be detected](./labeling_points)
- [x] [Coarse registration](./coarse_registration)
- [x] [Fine registration](./fine_registration)
- [x] [Searching the points to be measured in the specified point cloud](./search_points)
- [x] Combining all components

## Compiler

For now, all steps are testing and running on `Windows 10` with `visual studio 2017`.

## Development Kits

Download from [here](https://1drv.ms/u/s!AnRiouA_fmTVml5dwH_br_p1cM7R?e=WszIbd).

## Requirements

Each step should has its dependencies libraries, also might require the whole libs as follows.

| Library | Purpose | Official Website | Usage| Note |
| :---:         |     :---:     | :---: | :---:|:---:|
 | Eigen 3.3.7   |  Core geometric calculation     | [link](http://eigen.tuxfamily.org/index.php?title=Main_Page)    | header-only | |
| OpenSceneGraph 3.6.4  |  Visulization of point cloud     | [link](http://www.openscenegraph.org/)    | compile require | |
| nanoflann 3.6.4  | Visiting operation in point cloud     | [link](https://github.com/jlblancoc/nanoflann)   | header-only | |
| CGAL 5.0.2  (The Computational Geometry Algorithms Library) | Associating with core geometric calculation     | [link](https://www.cgal.org/)   | compile require| |
| OpenGR(A C++ library for 3D Global Registration)  |  Implement Super4PCS for coarse registration  | [link](https://github.com/STORM-IRIT/OpenGR) or [link](https://storm-irit.github.io/OpenGR/index.html)   | header-only | |
| libpointmatcher with its `yaml-cpp-pm`   | Implement  ICP  for fine registration    | [link](https://github.com/ethz-asl/libpointmatcher)   | compile require | [modified version](https://github.com/Gltina/libpointmatcher) that support export matrix per iteration instead of completed vtk-file when setting `dumpReading`|

## Demo

![demo](./demo/demo.gif)