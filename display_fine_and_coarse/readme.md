# display_fine_and_coarse

The purpose of  this program is to visualize the process of fine and coarse registration such that this is a separate tool.

## Usage

``` shell
display_fine_and_coarse.exe <point_cloud_1> <point_cloud_2> <output_folder>
```

Note that this program will find these file (as shown below) to read matrix in order to display point_cloud_1 with transformation.

``` shell
coarse file path:

output/coarse_matrix.txt

fine file path:

output//-matrix-0.txt
output//-matrix-1.txt
output//-matrix-2.txt
...
output//-matrix-16.txt

```

## Demo

![demo](./demo/coarse_fin_registration.gif)