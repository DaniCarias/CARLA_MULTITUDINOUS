# Quick Start
### Clone this repository & run the Carla Sim
```
git clone https://github.com/DaniCarias/Ground-Truth-Carla-Sim.git
cd [YOUR-PATH-TO-CARLA]
.\CarlaUE4.sh
```


## Generate RGB-D, LiDAR PCL, and voxel occupancy grid DataSets
```
python3 main.py
```

#### You can define...
* The frame interval for generating the DataSet (default = 750):
```
python3 main.py -f 1000
```
* The leaf size you want to downsample the Point Cloud (default = 0.2 (20cm)):  
```
python3 main.py -l 0.4
```
* If you want to generate traffic or not (default = 0):
```
python3 main.py -t 1
```
* The map (default = Town01_Opt):
```
python3 main.py -m Town02_Opt
```
* The route (default = route_1):
```
python3 main.py -r route_2
```

### To stop earlier
If you want to finish click on the `"Q"` key to destroy the actors and to avoid the risk of having a different number of samples for some type of data.


## Generate segmentation point clouds DataSets

```
python3 point_cloud_seg.py
```



## Visualize

### To visualize 20 random voxel occupancy grids:
Just change the path to get the voxel files in the visualize_voxel_grid.py code...
```
cd utils/visualize

python3 visualize_voxel_grid.py
```

### To visualize a point cloud:
Just change the path to get the point clouds in the visualize_point_cloud.py code...
```
cd utils/visualize
```

To view a LiDAR PCL:
```
python3 visualize_point_cloud.py -L 1
```
To view a segmentation PCL:
```
python3 visualize_point_cloud.py -S 1
```
To view a ground truth PCL:
```
python3 visualize_point_cloud.py -G 1
```

## Vehicle routes:
To create and build routes in Carla Sim

```
python3 vehicle_routes.py
```


