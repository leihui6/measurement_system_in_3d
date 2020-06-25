# coarse registration

The purpose of this stage is to find a rough transformation between two point cloud using [4PCS](https://github.com/STORM-IRIT/OpenGR) , which is a global registration method.

## Requirements

- OpenGR (i.e.  4-points Congruent Sets global registration algorithm)

## Usage

``` shell
./main_coarse_registration.exe  <point_cloud_1> <point_cloud_2> <output_folder>
```

Note that the final matrix will be written in `output/coarse_matrix.txt`

## Parameters in 4PCS

``` c++
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
```