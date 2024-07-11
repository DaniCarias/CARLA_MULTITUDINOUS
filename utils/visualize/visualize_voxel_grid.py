import numpy as np
import open3d as o3d
import glob

path = "../../../../../DataSets_vehicles_pedestrians_200x200x16/DayClear/test"

def main():

    ground_truth_samples = glob.glob(path+'/ground_truth/*')
    samples = np.random.choice(ground_truth_samples, 20, replace=False)

    for sample in samples:
        voxel_occupancy_grid = np.load(sample)['arr_0']
        occupied_indices = np.argwhere(voxel_occupancy_grid)
        
        occupied_coords = occupied_indices * 0.4 + np.array([0, 0, 0])
        pcl_voxel_grid = o3d.geometry.PointCloud()
        pcl_voxel_grid.points = o3d.utility.Vector3dVector(occupied_coords)
        # put with colors
        pcl_voxel_grid.colors = o3d.utility.Vector3dVector(np.random.rand(occupied_coords.shape[0], 3))
        #o3d.visualization.draw_geometries([pcl_voxel_grid])

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcl_voxel_grid,
                                                                    voxel_size=0.4)
        o3d.visualization.draw_geometries([pcl_voxel_grid])
        o3d.visualization.draw_geometries([voxel_grid])
    
if __name__ == "__main__":
    main()
