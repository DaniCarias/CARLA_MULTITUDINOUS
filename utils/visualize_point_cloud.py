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
        cloud1 = o3d.io.read_point_cloud(glob.glob('../_out/lidar/*.ply')[0])        # Lidar
        cloud2 = o3d.io.read_point_cloud(glob.glob('../_out/ground_truth/*.ply')[0]) # Ground Truth
        visualization.draw_geometries([cloud2])    # Visualize point cloud
        visualization.draw_geometries([cloud1])    # Visualize point cloud
    elif args.lidar:
        cloud = o3d.io.read_point_cloud(glob.glob('../_out/lidar/*.ply')[0])        # Lidar
        visualization.draw_geometries([cloud])    # Visualize point cloud
    elif args.segmentation:
        cloud = o3d.io.read_point_cloud(glob.glob('../_out/lidarSegm/*.ply')[0])    # Segmentation
        visualization.draw_geometries([cloud])    # Visualize point cloud
    
    

if __name__ == "__main__":
    main()