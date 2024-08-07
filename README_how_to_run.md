# Quick Start
### Clone this repository & run the Carla Sim
```
git clone https://github.com/DaniCarias/Ground-Truth-Carla-Sim.git
cd [YOUR-PATH-TO-CARLA]
.\CarlaUE4.sh
```


## Generate RGB-D, LiDAR PCL, and voxel occupancy grid DataSets
```
python3 main_dataset.py
```

#### You can define...
* The frame interval for generating the DataSet (default = 750):
```
python3 main_dataset.py -f 1000
```
* The leaf size you want to downsample the Point Cloud (default = 0.2 (20cm)):  
```
python3 main_dataset.py -l 0.4
```
* If you want to generate traffic or not (default = 0):
```
python3 main_dataset.py -t 1
```
* The map (default = Town01_Opt):
```
python3 main_dataset.py -m Town02_Opt
```
* The route (default = route_1):
```
python3 main_dataset.py -r route_2
```

### To stop earlier
If you want to finish click on the `"Q"` key to destroy the actors and to avoid the risk of having a different number of samples for some type of data.


## Generate segmentation point clouds DataSets

```
python3 point_cloud_seg.py
```


## Visualize

### To visualize the Ground Truth every frame of the simulation:

```
python3 ground_truth.py
```

### To visualize 1 random voxel occupancy grid:

```
cd utils/visualize

python3 visualize_voxel_grid.py
```

### To visualize a point cloud:

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

Visualize all the routes in the map to create routes in Carla Sim:
```
python3 vehicle_routes.py -a 1
```
To create one route, simply create an ordered array with the spawn_points chosen for the route.


Display a route (array of spawn_points) on the CARLA Sim:
```
python3 vehicle_routes.py
```
