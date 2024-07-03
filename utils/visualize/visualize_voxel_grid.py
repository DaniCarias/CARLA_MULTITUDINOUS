import numpy as np
import open3d as o3d
import glob

path = "../../../../../DataSets_vehicles_pedestrians/DayClear/test"

def main():

    ground_truth_samples = glob.glob(path+'/ground_truth/*')
    samples = np.random.choice(ground_truth_samples, 20, replace=False)

    for sample in samples:
        voxel_occupancy_grid = np.load(sample)['arr_0']
        occupied_indices = np.argwhere(voxel_occupancy_grid)
        occupied_coords = occupied_indices * 0.4 + np.array([0, 0, 0])
        pcl_voxel_grid = o3d.geometry.PointCloud()
        pcl_voxel_grid.points = o3d.utility.Vector3dVector(occupied_coords)
        o3d.visualization.draw_geometries([pcl_voxel_grid])

    
if __name__ == "__main__":
    main()
