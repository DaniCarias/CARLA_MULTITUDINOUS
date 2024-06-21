import numpy as np
import open3d as o3d
import glob
import os

def main(path):

    voxel_occupancy_grid = np.load(path+'/ground_truth/20240621_143332_000807.npz')['arr_0']
    occupied_indices = np.argwhere(voxel_occupancy_grid)
    occupied_coords = occupied_indices * 0.4 + np.array([0, 0, 0])
    pcl_voxel_grid = o3d.geometry.PointCloud()
    pcl_voxel_grid.points = o3d.utility.Vector3dVector(occupied_coords)
    
    
    pcl_ground_truth = o3d.io.read_point_cloud(path+'/ground_truth/20240621_143332_000807.ply')
    pcl_lidar = o3d.io.read_point_cloud(path+'/lidar/20240621_143332_000807.ply')
    
    
    o3d.visualization.draw_geometries([pcl_voxel_grid])
    o3d.visualization.draw_geometries([pcl_ground_truth])
    o3d.visualization.draw_geometries([pcl_lidar])
    
    
    #rbg_image = o3d.io.read_image('../../_out/rgb/20240619_145713_000288.png')
    #depth_image = o3d.io.read_image('../../_out/depth/20240619_145713_000288.png')
    #o3d.visualization.draw_geometries([pcl_ground_truth, pcl_voxel_grid, pcl_lidar])
    #o3d.visualization.draw([o3d.geometry.Image(rbg_image), o3d.geometry.Image(depth_image)])
    
    
def del_ply(path):
    
    ply_files = glob.glob(path)
    for ply_file in ply_files:
        os.remove(ply_file)
    
    
    

if __name__ == "__main__":
    
    #main("../../T1R1_750_NightCloudy")
    
    del_ply('../../T1R1_750_NightCloudy/ground_truth/*.ply')