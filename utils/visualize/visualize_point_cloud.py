import open3d as o3d
from open3d import visualization
import glob 
import argparse

parser = argparse.ArgumentParser(
    prog='Visualize Point Cloud',
)
parser.add_argument('--lidar', '-L', help='If you want to visualize the lidar point cloud')
parser.add_argument('--segmentation', '-S', help='If you want to visualize the segmentation point cloud')
parser.add_argument('--ground_truth', '-G', help='If you want to visualize the ground truth point cloud')

args = parser.parse_args()

def main():

    if args.ground_truth:
        lidar_cloud = o3d.io.read_point_cloud(glob.glob('../../_out/lidar/*.ply')[0])        # Lidar
        ground_truth_cloud = o3d.io.read_point_cloud(glob.glob('../../_out/ground_truth/*.ply')[0]) # Ground Truth
        visualization.draw_geometries([ground_truth_cloud])    # Visualize point cloud
        visualization.draw_geometries([lidar_cloud])    # Visualize point cloud
    elif args.lidar:
        lidar_cloud = o3d.io.read_point_cloud(glob.glob('../../_out/lidar/*.ply')[0])        # Lidar
        visualization.draw_geometries([lidar_cloud])    # Visualize point cloud
    elif args.segmentation:
        lidarSegm_cloud = o3d.io.read_point_cloud(glob.glob('../../_out/lidarSegm/*.ply')[0])    # Segmentation
        visualization.draw_geometries([lidarSegm_cloud])    # Visualize point cloud
    
    

if __name__ == "__main__":
    main()