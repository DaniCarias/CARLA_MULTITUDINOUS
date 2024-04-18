import open3d as o3d
import os

def convert_image_to_point_cloud(depth_image_path, rgb_image_path):

  color = o3d.io.read_image(rgb_image_path)
  depth = o3d.io.read_image(depth_image_path)

  rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
  
  point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
      rgbd,
      o3d.camera.PinholeCameraIntrinsic(
          width=1280,
          height=720,
          intrinsic_matrix=[[0.08, 0, 640],
                            [0, 0.08, 360],
                            [0, 0, 1]]
      )
  )
  return point_cloud

def main():
  """Converts all depth images in a folder to a single point cloud and saves it as a .ply file."""

  depth_image_folder = "../_out/depth"
  rgb_image_folder = "../_out/rgb"
  
  camera_intrinsics = {  # Replace with your camera intrinsics
      "focal_length_x": 0.08,
      "focal_length_y": 0.08,
      "principal_point_x": 640,
      "principal_point_y": 360
  }

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