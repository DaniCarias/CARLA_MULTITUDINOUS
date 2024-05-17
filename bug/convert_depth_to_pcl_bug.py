import open3d as o3d
import math as mt
import numpy as np
import os
import cv2
import time
import pcl

depth_0_folder = "../_out/ground_truth/degree_0/depth"
rgb_0_folder = "../_out/ground_truth/degree_0/rgb"
depth_60_folder = "../_out/ground_truth/degree_60/depth"
rgb_60_folder = "../_out/ground_truth/degree_60/rgb"
depth_120_folder = "../_out/ground_truth/degree_120/depth"
rgb_120_folder = "../_out/ground_truth/degree_120/rgb"
depth_180_folder = "../_out/ground_truth/degree_180/depth"
rgb_180_folder = "../_out/ground_truth/degree_180/rgb"
depth_240_folder = "../_out/ground_truth/degree_240/depth"
rgb_240_folder = "../_out/ground_truth/degree_240/rgb"
depth_300_folder = "../_out/ground_truth/degree_300/depth"
rgb_300_folder = "../_out/ground_truth/degree_300/rgb"


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

def concatenate_pcl(pcl_0, pcl_60, pcl_120, pcl_180, pcl_240, pcl_300):
  
    # Get the points of the point clouds
    p0_points = np.asarray(pcl_0.points)
    p60_points = np.asarray(pcl_60.points)
    p120_points = np.asarray(pcl_120.points)
    p180_points = np.asarray(pcl_180.points)
    p240_points = np.asarray(pcl_240.points)
    p300_points = np.asarray(pcl_300.points)
    # Get the colors of the point clouds
    p0_colors = np.asarray(pcl_0.colors)
    p60_colors = np.asarray(pcl_60.colors)
    p120_colors = np.asarray(pcl_120.colors)
    p180_colors = np.asarray(pcl_180.colors)
    p240_colors = np.asarray(pcl_240.colors)
    p300_colors = np.asarray(pcl_300.colors)
    
    # Concatenate the points and colors
    p360_points = np.concatenate((p0_points, p60_points, p120_points, p180_points, p240_points, p300_points), axis=0)
    p360_colors = np.concatenate((p0_colors, p60_colors, p120_colors, p180_colors, p240_colors, p300_colors), axis=0)
    
    # Create the point cloud with the concatenated points and colors
    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(p360_points)
    pcl.colors = o3d.utility.Vector3dVector(p360_colors)

    return pcl



def convert_imgs_to_pcl_manualy(depth_path_0, imgSizeX=800, imgSizeY=600, fov=90):

    # Intrinsics of the camera
    focal_lengthX = imgSizeX / (2 * mt.tan(fov * mt.pi / 360))
    #focal_lengthY = imgSizeY / (2 * mt.tan(fov * mt.pi / 360))
    centerX = imgSizeX / 2
    centerY = imgSizeY / 2

    intrinsic_matrix = [[focal_lengthX, 0, centerX],
                        [0, focal_lengthX, centerY],
                        [0, 0, 1]]
    
    print(f"{intrinsic_matrix}")
    
    intrinsic_matrix_inv = np.linalg.inv(intrinsic_matrix)
    
    depth_img = cv2.imread(depth_path_0, cv2.IMREAD_UNCHANGED)
    
    points_3d = []
    
    for v in range(imgSizeY):
        for u in range(imgSizeX):            
            # Depth of the point
            depth = depth_img[v, u]
            #print(depth)
            
            """ im = np.int32(depth_img)
            #print(im[v,u])
            
            in_meters = np.double( im[v,u] + (im[v,u] * 256) + (im[v,u] * 256 * 256) ) / np.double((256 * 256 * 256) - 1) * 1000
            #print(f"{depthMatrix}")
            
            #normalized = (rgb_value[0] + rgb_value[1] * 256 + rgb_value[2] * 256 * 256) / (256 * 256 * 256 - 1)
            #in_meters = 1000 * normalized """
            
            p2d = np.array([u, v, 1])
            
            # Get the 3D point
            P = np.dot(intrinsic_matrix_inv, p2d) * depth

            points_3d.append(P)

    # Create a point cloud from numpy array
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(np.array(points_3d))
    
    # Visualize
    o3d.visualization.draw_geometries([point_cloud])
    
    


def convert_imgs_to_pcl(depth_0_folder, rgb_0_folder, depth_60_folder, rgb_60_folder, depth_120_folder, rgb_120_folder, depth_180_folder, rgb_180_folder, depth_240_folder, rgb_240_folder, depth_300_folder, rgb_300_folder,
                        imgSizeX=1280, imgSizeY=720, fov=60):
    
    # Intrinsics of the camera
    focal_lengthX = imgSizeX / (2 * mt.tan(fov * mt.pi / 360))
    #focal_lengthY = imgSizeY / (2 * mt.tan(fov * mt.pi / 360))
    centerX = imgSizeX / 2
    centerY = imgSizeY / 2

    intrinsic_matrix = [[focal_lengthX, 0, centerX],
                        [0, focal_lengthX, centerY],
                        [0, 0, 1]]

    # Get the color and depth images
    rgb_0 = o3d.io.read_image(rgb_0_folder)
    depth_0 = o3d.io.read_image(depth_0_folder)
    rgb_60 = o3d.io.read_image(rgb_60_folder)
    depth_60 = o3d.io.read_image(depth_60_folder)
    rgb_120 = o3d.io.read_image(rgb_120_folder)
    depth_120 = o3d.io.read_image(depth_120_folder)
    rgb_180 = o3d.io.read_image(rgb_180_folder)
    depth_180 = o3d.io.read_image(depth_180_folder)
    rgb_240 = o3d.io.read_image(rgb_240_folder)
    depth_240 = o3d.io.read_image(depth_240_folder)
    rgb_300 = o3d.io.read_image(rgb_300_folder)
    depth_300 = o3d.io.read_image(depth_300_folder)

    # Create the RGBD image
    rgbd_0 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_0, depth_0, convert_rgb_to_intensity = False)
    rgbd_60 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_60, depth_60, convert_rgb_to_intensity = False)
    rgbd_120 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_120, depth_120, convert_rgb_to_intensity = False)
    rgbd_180 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_180, depth_180, convert_rgb_to_intensity = False)
    rgbd_240 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_240, depth_240, convert_rgb_to_intensity = False)
    rgbd_300 = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_300, depth_300, convert_rgb_to_intensity = False)
    
    # Create the point clouds from the RGBD images
    pcl_0 = create_pcl_from_rgbd(rgbd_0, intrinsic_matrix)
    pcl_60 = create_pcl_from_rgbd(rgbd_60, intrinsic_matrix)
    pcl_120 = create_pcl_from_rgbd(rgbd_120, intrinsic_matrix)
    pcl_180 = create_pcl_from_rgbd(rgbd_180, intrinsic_matrix)
    pcl_240 = create_pcl_from_rgbd(rgbd_240, intrinsic_matrix)
    pcl_300 = create_pcl_from_rgbd(rgbd_300, intrinsic_matrix)
    
    
    # Rotate the pcl_60 60 degrees
    R = pcl_60.get_rotation_matrix_from_xyz((0, 60 * mt.pi / 180, 0))
    pcl_60 = pcl_60.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_120 120 degrees
    R = pcl_120.get_rotation_matrix_from_xyz((0, 120 * mt.pi / 180, 0))
    pcl_120 = pcl_120.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_180 180 degrees
    R = pcl_180.get_rotation_matrix_from_xyz((0, 180 * mt.pi / 180, 0))
    pcl_180 = pcl_180.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_240 240 degrees
    R = pcl_240.get_rotation_matrix_from_xyz((0, 240 * mt.pi / 180, 0))
    pcl_240 = pcl_240.rotate(R, center=(0,0,0))
    
    # Rotate the pcl_300 300 degrees
    R = pcl_300.get_rotation_matrix_from_xyz((0, 300 * mt.pi / 180, 0))
    pcl_300 = pcl_300.rotate(R, center=(0,0,0))
    
    # Concatenate the point clouds  
    pcl = concatenate_pcl(pcl_0, pcl_60, pcl_120, pcl_180, pcl_240, pcl_300)
    
    #pcl = pcl.remove_radius_outlier(nb_points=16, radius=75.0)
  
    return pcl


def main():

    # Get the rgb and depth images in the same order
    for file in os.listdir(depth_0_folder):
        depth_path_0 = os.path.join(depth_0_folder, file)
        rgb_path_0 = os.path.join(rgb_0_folder, file)
        depth_path_60 = os.path.join(depth_60_folder, file)
        rgb_path_60 = os.path.join(rgb_60_folder, file)
        depth_path_120 = os.path.join(depth_120_folder, file)
        rgb_path_120 = os.path.join(rgb_120_folder, file)
        depth_path_180 = os.path.join(depth_180_folder, file)
        rgb_path_180 = os.path.join(rgb_180_folder, file)
        depth_path_240 = os.path.join(depth_240_folder, file)
        rgb_path_240 = os.path.join(rgb_240_folder, file)
        depth_path_300 = os.path.join(depth_300_folder, file)
        rgb_path_300 = os.path.join(rgb_300_folder, file)
        
        #### Formula manual
        convert_imgs_to_pcl_manualy(depth_path_0)
        
        
        #pcl = convert_imgs_to_pcl(depth_path_0, rgb_path_0, depth_path_60, rgb_path_60, depth_path_120, rgb_path_120, depth_path_180, rgb_path_180, depth_path_240, rgb_path_240, depth_path_300, rgb_path_300)

        # Save with timestamp HHMMSS
        #timestamp = time.strftime("%H%M%S")
        #o3d.io.write_point_cloud(f"../_out/ground_truth/point_clouds/{timestamp}.pcd", pcl)

        #pcl.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

        o3d.visualization.draw_geometries([pcl])
    
    

    #o3d.io.write_point_cloud("../_out/ground_truth/depth1/output.ply", point_cloud)  # Save as a PLY file

if __name__ == "__main__":
    main()