from open3d import visualization, io  
import glob 

def main():
    # get the first .ply in the folder "_out/lidar"
    #cloud = io.read_point_cloud(glob.glob('../_out/lidar/*.ply')[0]) # Read point cloud
    #cloud = io.read_point_cloud(glob.glob('../_out/lidar_teste/*.ply')[30]) # Read point cloud
    cloud = io.read_point_cloud(glob.glob('../_out/lidarSegm/*.ply')[1])
    visualization.draw_geometries([cloud])    # Visualize point cloud

if __name__ == "__main__":
    main()