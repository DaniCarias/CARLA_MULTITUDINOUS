from open3d import *   
import glob 

def main():
    # get the first .ply in the folder "_out/lidar"
    cloud = io.read_point_cloud(glob.glob('../_out/lidar/*.ply')[1]) # Read point cloud
    visualization.draw_geometries([cloud])    # Visualize point cloud      

if __name__ == "__main__":
    main()