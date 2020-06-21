# searching points

This goal of this program is to search points in reference point cloud besed on poits marked in reading point cloud.

## Requirements

- CGAL 5.0.2

## Usage

``` shell
./search_points.exe  <point_cloud_1> <point_cloud_2> <output_folder>
```

This program should be executed after fine and coarse registration because it will load marked/fine matrix/ coarse matrix file in `<output_folder>`.
