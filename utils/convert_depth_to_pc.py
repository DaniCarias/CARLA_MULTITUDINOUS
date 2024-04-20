import open3d as o3d
import math as mt
import numpy as np
import os

def create_pcl_from_rgbd(rgbd_image, intrinsic, width=1280, height=720):
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            width = width,
            height = height,
            intrinsic_matrix = intrinsic
        )
    )
    return point_cloud
  
def concatenate_pcl(pcl_0, pcl_120, pcl_240):
  
    # Get the points of the point clouds
    p0_points = np.asarray(pcl_0.points)
    p120_points = np.asarray(pcl_120.points)
    p240_points = np.asarray(pcl_240.points)
    # Get the colors of the point clouds
    p0_colors = np.asarray(pcl_0.colors)
    p120_colors = np.asarray(pcl_120.colors)
    p240_colors = np.asarray(pcl_240.colors)
    
    # Concatenate the points and colors
    p360_points = np.concatenate((p0_points, p120_points, p240_points), axis=0)
    p360_colors = np.concatenate((p0_colors, p120_colors, p240_colors), axis=0)
    
    # Create the point cloud with the concatenated points and colors
    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(p360_points)
    pcl.colors = o3d.utility.Vector3dVector(p360_colors)

    return pcl




def convert_imgs_to_pcl(depth_path_0, rgb_path_0, depth_path_120, rgb_path_120, depth_path_240, rgb_path_240, imgSizeX=1280, imgSizeY=720, fov=120):
    
    # Intrinsics of the camera
    focus_length = imgSizeX / (2 * mt.tan(fov * mt.pi / 360))
    centerX = imgSizeX / 2
    centerY = imgSizeY / 2
    intrinsic_matrix = [[focus_length, 0, centerX],
                        [0, focus_length, centerY],
                        [0, 0, 1]]

    # Get the color and depth images
    rgb_0 = o3d.io.read_image(rgb_path_0)
    depth_0 = o3d.io.read_image(depth_path_0)
    rgb_120 = o3d.io.read_image(rgb_path_120)
    depth_120 = o3d.io.read_image(depth_path_120)
    rgb_240 = o3d.io.read_image(rgb_path_240)
    depth_240 = o3d.io.read_image(depth_path_240)

    
    # Create the RGBD image
    rgbd_0 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_0, depth_0, convert_rgb_to_intensity = False)
    rgbd_120 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_120, depth_120, convert_rgb_to_intensity = False)
    rgbd_240 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_240, depth_240, convert_rgb_to_intensity = False)
    
    # Create the point clouds from the RGBD images
    pcl_0 = create_pcl_from_rgbd(rgbd_0, intrinsic_matrix)
    pcl_120 = create_pcl_from_rgbd(rgbd_120, intrinsic_matrix)
    pcl_240 = create_pcl_from_rgbd(rgbd_240, intrinsic_matrix)
    
    # Rotate the pcl_120 120 degrees
    R = pcl_120.get_rotation_matrix_from_xyz((0, 120 * mt.pi / 180, 0))
    pcl_120 = pcl_120.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_240 240 degrees
    R = pcl_240.get_rotation_matrix_from_xyz((0, 240 * mt.pi / 180, 0))
    pcl_240 = pcl_240.rotate(R, center=(0,0,0))
    
    # Concatenate the point clouds  
    pcl = concatenate_pcl(pcl_0, pcl_120, pcl_240)
    
    #pcl = pcl.remove_radius_outlier(nb_points=16, radius=75.0)
  
    return pcl


def main():

    depth_0_folder = "../_out/ground_truth/degree_0/depth"
    rgb_0_folder = "../_out/ground_truth/degree_0/rgb"
    depth_120_folder = "../_out/ground_truth/degree_120/depth"
    rgb_120_folder = "../_out/ground_truth/degree_120/rgb"
    depth_240_folder = "../_out/ground_truth/degree_240/depth"
    rgb_240_folder = "../_out/ground_truth/degree_240/rgb"
    
    
    # Get the rgb and depth images in the same order
    for file in os.listdir(depth_0_folder):        
        depth_path_0 = os.path.join(depth_0_folder, file)
        rgb_path_0 = os.path.join(rgb_0_folder, file)
        depth_path_120 = os.path.join(depth_120_folder, file)
        rgb_path_120 = os.path.join(rgb_120_folder, file)
        depth_path_240 = os.path.join(depth_240_folder, file)
        rgb_path_240 = os.path.join(rgb_240_folder, file)
        
        pcl = convert_imgs_to_pcl(depth_path_0, rgb_path_0, depth_path_120, rgb_path_120, depth_path_240, rgb_path_240)

        pcl.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        o3d.visualization.draw_geometries([pcl])
    

    #o3d.io.write_point_cloud("../_out/ground_truth/depth1/output.ply", point_cloud)  # Save as a PLY file

if __name__ == "__main__":
    main()