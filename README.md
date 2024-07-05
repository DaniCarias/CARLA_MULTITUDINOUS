<!-- ABOUT THE PROJECT -->
## About The Project

The main objective of this project is to generate datasets for machine learning with RGB images, depth images, point clouds from a LiDAR, and voxel occupancy grids to ground truth.

**This repository explores:**
* The creation of ground truth of the 3D environment in a voxel occupancy grid, based on depth images.
* Fix the LiDAR coordinates system.
* Fix the alignment of the PCL from LiDAR with the ground truth
* Creating routes for the vehicle to travel
* Creating datasets with point clouds of segmentation
* Creating datasets with RGB-D, LiDAR PCL and ground truth


## Create the ground truth from depth images

The method chosen to obtain the ground truth was based on depth images. 
Since the Carla Sim does not allow for a 360º FOV in the depth camera, it was decided to use 4 cameras with a 90º FOV. In this way, the 4 cameras are positioned to cover 360º.

#### Intrinsic parameters
To transform the depth image into a point cloud, it was necessary to obtain the camera's intrinsic parameters to convert the coordinates of the image pixels into 3D coordinates. These intrinsic parameters include the focal length, optical center, and distortion coefficients, and are essential for calculating the exact position of each point in 3D space from the depth data.

#### Extrinsic parameters
To describe the position and orientation of the camera in 3D space, we needed the extrinsic parameters, which provide information on how the camera is positioned and oriented about the coordinate system. The extrinsic matrix contains information about the rotation and translation of the points in the point cloud.

#### Get the depth (Z axis)
To obtain the normalized depth [0, 1] of each pixel, of each image, the mathematical function was used:

$P(u,v) = \frac{R + G * 256 + B * 256 * 256}{256 * 256 * 256 - 1}$

Next, the depth values were converted to meters, and the points with depths of less than 90 meters were selected.

#### Get the (X, Y, Z) coordinates
From the respective depth values and the intrinsic matrix of the matrix, they were converted into 3D points (X, Y, and Z).
To transform the points into 3 dimensions, the following mathematical function is applied the following mathematical function:

![image](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/021c26f8-1af7-4e2f-ad72-acaabd7ea055)

Where "x" and "y" are the coordinates of the image pixels, "fl" is the focal length, "c" is the center of the image, and "p" are the previously calculated depth values corresponding to each pixel.

The result of these operations is a matrix of points with the coordinates x, y, and z.

#### Camera to word
To locate the 4 point clouds in 3D space, to create a 360º view, we had to multiply the resulting matrix by the extrinsic matrix of each camera to obtain the rotation and translation that each point cloud needs to be correctly located about its origin (camera).


**Obtaining the following result:**
![ground_truth_90](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/b41fc829-3acb-4481-866f-9153c04876b5)
![ground_truth_360](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/cc2cc3c0-5765-4405-96bd-f38ffb996033)

Based on this [issue](https://github.com/carla-simulator/carla/issues/6719)



## Fix the LiDAR coordinates system

The point cloud obtained from the LiDAR sensor does not follow the same coordinate system as the other sensors and cameras, since it follows the right-hand coordinate system, and the world in the simulator itself follows the left-hand system, causing the two point clouds (LiDAR and ground truth) to be "mirrored" on the Y axis.

![image](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/3df6cfd4-53de-40e7-a6fd-b5445f7b1998)

To solve this problem, translations and rotations are made in the LiDAR point cloud to replicate the transformation (x, y, z) → (y, x, -z).

Based on this [issue](https://github.com/carla-simulator/carla/issues/392)



## Fix the alignment of the PCL from LiDAR with the ground truth

The ground truth point cloud is generated from 4 depth cameras, theoretically positioned in the same location, guaranteeing a common spatial reference.
  
But in practice, each 90º point cloud has its origin slightly forward of the location indicated as its origin, meaning that the point cloud does not have its origin in that location.

It was therefore necessary to obtain the theoretical center of the four cameras. By averaging the coordinates of each origin, the supposed real center of the ground truth point cloud is obtained. The figure shows the 4 supposed origins of the cameras and the central point that represents the true origin, obtained from the average.

![pontos_camaras](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/23c72103-a358-4779-87e5-1d57d00f87d3)



## Creating routes for the vehicle to travel

In order to be able to compare the results obtained in different types of datasets, more or less complex, specific routes were created for the vehicle to circulate, so that we can change the weather, for example, and see, from the results obtained, how much this factor penalizes the model's performance or not, without being influenced by the route the vehicle takes.

![image](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/980f84a5-a5ee-41c6-bd63-10f8beb08faa)

With access to the Carla Sim documentation, it is possible to view all the spawn points on the map, so all you have to do is build an ordered array with the numbers of the spawn points you want the vehicle to pass.

Like this:

![Town1_routes](https://github.com/DaniCarias/Carla_sim_proj_infor/assets/93714772/4587f013-29d8-4193-9fcb-dd7383b13898)

Based on Carla Sim [documentation](https://carla.readthedocs.io/en/0.9.14/tuto_G_traffic_manager/) in "Specify routes for vehicles" section








