import open3d as o3d
import math as mt
import os

def convert_image_to_point_cloud(depth_image_path, rgb_image_path, imgSizeX=1280, imgSizeY=720, fov=110):
  # Intrinsics of the camera
  focus_length = imgSizeX / (2 * mt.tan(fov * mt.pi / 360))
  centerX = imgSizeX / 2
  centerY = imgSizeY / 2

  # Get the color and depth images
  color = o3d.io.read_image(rgb_image_path)
  depth = o3d.io.read_image(depth_image_path)
  # Create the RGBD image
  rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
  
  # Create the point cloud
  point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
      rgbd,
      o3d.camera.PinholeCameraIntrinsic(
          width=1280,
          height=720,
          intrinsic_matrix=[[focus_length, 0, centerX],
                            [0, focus_length, centerY],
                            [0,            0,       1]]
      )
  )
  return point_cloud

def main():

  depth_image_folder = "../_out/depth"
  rgb_image_folder = "../_out/rgb"
  
  point_cloud = o3d.geometry.PointCloud()
  #for filename in os.listdir(depth_image_folder):
    #if filename.endswith(".png"):  # Adjust for your image format
    
  depth_image_path = os.path.join(depth_image_folder, "20240419_002403_011756.png")
  rgb_image_path = os.path.join(rgb_image_folder, "20240419_002402_011756.png")
  
  point_cloud = convert_image_to_point_cloud(depth_image_path, rgb_image_path)


  point_cloud.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

  o3d.visualization.draw_geometries([point_cloud])

  #o3d.io.write_point_cloud("../_out/ground_truth/depth1/output.ply", point_cloud)  # Save as a PLY file

if __name__ == "__main__":
  main()