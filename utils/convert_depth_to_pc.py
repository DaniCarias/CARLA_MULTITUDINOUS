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


def convert_imgs_to_pcl(imgs_360, imgSizeX=1280, imgSizeY=720, fov=110):
    
    # Intrinsics of the camera
    focus_length = imgSizeX / (2 * mt.tan(fov * mt.pi / 360))
    centerX = imgSizeX / 2
    centerY = imgSizeY / 2
    intrinsic_matrix = [[focus_length, 0, centerX],
                        [0, focus_length, centerY],
                        [0, 0, 1]]

    #print(imgs_360)
    # Get the color and depth images
    rgb_0 = o3d.io.read_image(imgs_360[0])
    depth_0 = o3d.io.read_image(imgs_360[1])
    rgb_120 = o3d.io.read_image(imgs_360[2])
    depth_120 = o3d.io.read_image(imgs_360[3])
    rgb_240 = o3d.io.read_image(imgs_360[4])
    depth_240 = o3d.io.read_image(imgs_360[5])
    
    # view the images
    #o3d.visualization.draw_geometries([rgb_0, depth_0, rgb_120, depth_120, rgb_240, depth_240])
    
    # Create the RGBD image
    rgbd_0 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_0, depth_0, convert_rgb_to_intensity = False)
    rgbd_120 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_120, depth_120, convert_rgb_to_intensity = False)
    rgbd_240 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_240, depth_240, convert_rgb_to_intensity = False)
    
    # Create the point clouds
    pcl_0 = create_pcl_from_rgbd(rgbd_0, intrinsic_matrix)
    pcl_120 = create_pcl_from_rgbd(rgbd_120, intrinsic_matrix)
    pcl_240 = create_pcl_from_rgbd(rgbd_240, intrinsic_matrix)
    
    # Rotate the pcl_120 120 degrees
    R = pcl_120.get_rotation_matrix_from_xyz((0, 120 * mt.pi / 180, 0))
    pcl_120 = pcl_120.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_240 240 degrees
    R = pcl_240.get_rotation_matrix_from_xyz((0, 240 * mt.pi / 180, 0))
    pcl_240 = pcl_240.rotate(R, center=(0,0,0))
    
    return pcl_0, pcl_120, pcl_240

def main():

    depth_0_folder = "../_out/ground_truth/degree_0/depth"
    rgb_0_folder = "../_out/ground_truth/degree_0/rgb"
    depth_120_folder = "../_out/ground_truth/degree_120/depth"
    rgb_120_folder = "../_out/ground_truth/degree_120/rgb"
    depth_240_folder = "../_out/ground_truth/degree_240/depth"
    rgb_240_folder = "../_out/ground_truth/degree_240/rgb"
    
    imgs_360 = []
    
    # Get the rgb and depth images in the same order
    for file in os.listdir(depth_0_folder):        
        depth_path_0 = os.path.join(depth_0_folder, file)
        rgb_path_0 = os.path.join(rgb_0_folder, file)
        depth_path_120 = os.path.join(depth_120_folder, file)
        rgb_path_120 = os.path.join(rgb_120_folder, file)
        depth_path_240 = os.path.join(depth_240_folder, file)
        rgb_path_240 = os.path.join(rgb_240_folder, file)
        
        imgs_360.append([rgb_path_0, depth_path_0, rgb_path_120, depth_path_120, rgb_path_240, depth_path_240])
    
    # Convert all the images to point clouds
    for i in range(len(imgs_360)):
      pcl_0, pcl_120, pcl_240 = convert_imgs_to_pcl(imgs_360[i])

      pcl_0.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
      pcl_120.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
      pcl_240.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

      o3d.visualization.draw_geometries([pcl_0, pcl_120, pcl_240])
    
    


    

    #o3d.io.write_point_cloud("../_out/ground_truth/depth1/output.ply", point_cloud)  # Save as a PLY file

if __name__ == "__main__":
    main()