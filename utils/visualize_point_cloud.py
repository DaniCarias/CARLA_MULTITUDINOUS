import open3d as o3d
from open3d import visualization
import glob 

GROUND_TRUTH = '../_out/ground_truth/'

def main():
    # get the first .ply in the folder "_out/lidar"
    
    #cloud = io.read_point_cloud(glob.glob('../_out/lidar/*.ply')[0])         # Lidar
    #cloud = io.read_point_cloud(glob.glob('../_out/lidarSegm/*.ply')[1])     # Segmentation
    #cloud = io.read_point_cloud('../_out/ground_truth/ground_truth.ply')     # Ground truth
    
# Convert .pcd to .ply and visualize point cloud filtred
    print("Reading .pcd file...")
    pcd = o3d.io.read_point_cloud(f"{GROUND_TRUTH}ground_truth_filtered.pcd")
    print(f"PCD: {pcd}")
    print("Writing .ply file...")
    o3d.io.write_point_cloud(f"{GROUND_TRUTH}ground_truth_filtered.ply", pcd)
    cloud = o3d.io.read_point_cloud(f"{GROUND_TRUTH}ground_truth_filtered.ply")
    
    visualization.draw_geometries([cloud])    # Visualize point cloud

if __name__ == "__main__":
    main()