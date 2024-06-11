import argparse
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
from utils.ground_truth import ground_truth_w_out_colors as ground_truth
import carla
import queue
import cv2
import numpy as np
import open3d as o3d
import time

parser = argparse.ArgumentParser(description="Carla Dataset")
parser.add_argument('-l', '--leaf_size', type=float, help='Leaf size for downsampling', default=0.1)
args = parser.parse_args()


lidar_attributes = {
    "real_lidar": {
        'channels': '128',                      # Numero de lasers -> 128
        'range': '75.0',                        # Distancia máxima em metros -> 75.0
        'points_per_second': '2621440',         # Pontos por segundo
        'rotation_frequency': '20',             # Velocidade de rotação Hz -> 20
        'upper_fov': '45',                      # Qual o ângulo do centro para cima
        'lower_fov': '-45',                     # Qual o ângulo do centro para baixo
        'dropoff_general_rate': '0.1',          # Qual a percentagem de pontos que vai descartar -> 0.1
        'dropoff_intensity_limit': '0.8',       # O que for a baixo do valor de intensidade é descartado -> 0.8
        'dropoff_zero_intensity': '0.4',        # Se o valor de intensidade for 0 qual a percentagem de descaratar -> 0.4
        'atmosphere_attenuation_rate': '0.4',   # Atenuação da atmosfera -> 0.4
    },
}

camera_attributes = {
    "real_rgb": {
        'fov': '110',                  # Field of view horizontal em graus 110 -> Baseado na camera Zed   
        'fstop': '0.5',                # Abertura da lente -> 1/2 -> Baseado na camera Zed
        'iso': '1900',                 # Sensitivity do sensor -> Baseado na camera Zed
        'focal_distance': '2000',      # Distância a que a profundidade do efeito de campo deve ser nítida.     
        #'motion_blur_intensity': '0',  # Intensidade do efeito de movimento
    },
        
    "rgb_and_depth": {
        'image_size_x': '1280',
        'image_size_y': '720',
        'fov': '110',                  # Field of view horizontal em graus 110 -> Baseado na camera Zed
        'lens_k': '0',                 # Remove the distortion
        'lens_kcube': '0',             # Remove the distortion
    },
}


def lidar_transformation(extrinsic, raw_data):
    """
    The function 'lidar_transformation' transforms raw lidar data into a point cloud, applies various
    rotations and translations to fit into ground truth point cloud.
    
    :param extrinsic: Represents the extrinsic calibration matrix that describes the transformation between 
                      the lidar sensor and the camera coordinate systems. It is used to calculate the rotation
                      around the Z-axis based on the ground truth camera data.
    :param raw_data: Is a byte array that contains the raw lidar data.
    
    :return: The function 'lidar_transformation' returns two values:
    1. 'lidar_pcl': An Open3D PointCloud object that represents the transformed lidar point cloud.
    2. 'center_lidar': A numpy array containing the coordinates of the origin of the lidar point cloud after transformation.
    """
    
    lidar_pcl = o3d.geometry.PointCloud()
    
    point_cloud_array = np.frombuffer(raw_data, dtype=np.float32)
    point_cloud_array = np.reshape(point_cloud_array, (int(point_cloud_array.shape[0] / 4), 4))            
    point_cloud_array = np.delete(point_cloud_array, 3, 1)
    
    
# Fix the lidar point cloud transformation to world coordinates
    yaw_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])   # Yaw = 90º
    point_cloud_array = np.dot(point_cloud_array, yaw_90) 
    
    point_cloud_array[:, 2] = -point_cloud_array[:, 2]      # Z = -Z

    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
# Transform the point cloud to the camera coordinate system
    # Rotation of 180 in the X axis
    x_180 = np.array([[1,  0,  0],
                      [0, -1,  0],
                      [0,  0, -1]])
    lidar_pcl.rotate(x_180, center=(0, 0, 0)) 
    
    # Rotation of 180 in the Y axis
    y_180 = np.array([[-1,  0,  0],
                      [ 0, -1,  0],
                      [ 0,  0,  1]])
    lidar_pcl.rotate(y_180, center=(0, 0, 0)) 
    
    # Rotation of (Extract rotation around Z-axis of the ground truth camera) in the Z axis
    theta_z = np.arctan2(extrinsic[1, 0], extrinsic[0, 0])
    z_rotation = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(z_rotation, center=(0, 0, 0))
    

    point_cloud_color = np.full((len(lidar_pcl.points),3), np.array([0,0,255]))  # All points are BLUE
    point_cloud_array = np.vstack([np.array(lidar_pcl.points), [0, 0, 0]])       # Add point 0,0,0 (origin)
    point_cloud_color = np.vstack([point_cloud_color, [255, 0, 0]])              # The (0,0,0) point is RED
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(point_cloud_array), axis=0)
    point_cloud_array = point_cloud_array - centroid
    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    lidar_pcl.colors = o3d.utility.Vector3dVector(point_cloud_color)
        
    center_lidar = point_cloud_array[-1] # Get the origin coordinates of the lidar point cloud
    
    return lidar_pcl, center_lidar
        
        
def get_ground_truth(queue_list, depth_camera_list):
    """
    The function 'get_ground_truth' processes depth images from multiple cameras to generate a point cloud.
    
    :param queue_list: Dictionary containing queues for different types of depth images.
    :param depth_camera_list: Dictionary containing depth camera objects for front, right, left, and back cameras.
    
    :return: The function `get_ground_truth` returns three values:
    1. `points`: A numpy array containing the 3D points in the world space for all four cameras.
    2. `colors`: A numpy array containing the color information (RGB) corresponding to each 3D point.
    3. `front_extrinsic_matrix`: The extrinsic matrix corresponding to the front camera.
    """
        
    front_depth_image = queue_list['image_queue_depth_front'].get()
    right_depth_image = queue_list['image_queue_depth_right'].get()
    left_depth_image = queue_list['image_queue_depth_left'].get()
    back_depth_image = queue_list['image_queue_depth_back'].get()
    
    # Show the RGB image
    front_rbg_image = queue_list['image_queue_rgb_front'].get()
    front_rbg_image = np.reshape(np.copy(front_rbg_image.raw_data), (front_rbg_image.height, front_rbg_image.width, 4))
    cv2.imshow('RGB Camera Front Output', front_rbg_image) 

    # Get the intrinsic and extrinsic matrix of the 4 cameras
    front_intrinsic_matrix, front_extrinsic_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['front_depth_camera'], front_depth_image)
    right_intrinsic_matrix, right_extrinsic_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['right_depth_camera'], right_depth_image)
    left_intrinsic_matrix, left_extrinsic_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['left_depth_camera'], left_depth_image)
    back_intrinsic_matrix, back_extrinsic_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['back_depth_camera'], back_depth_image)

    # Get the points [[X...], [Y...], [Z...]] and the colors [[R...], [G...], [B...]]
    front_points_3D, front_color = ground_truth.point2D_to_point3D(front_depth_image, front_intrinsic_matrix)
    right_points_3D, right_color = ground_truth.point2D_to_point3D(right_depth_image, right_intrinsic_matrix)
    left_points_3D, left_color = ground_truth.point2D_to_point3D(left_depth_image, left_intrinsic_matrix)
    back_points_3D, back_color = ground_truth.point2D_to_point3D(back_depth_image, back_intrinsic_matrix)

    # To multiply by the extrinsic matrix (same shape as the extrinsic_matrix matrix)
    front_p3d = np.concatenate((front_points_3D, np.ones((1, front_points_3D.shape[1]))))
    right_p3d = np.concatenate((right_points_3D, np.ones((1, right_points_3D.shape[1]))))
    left_p3d = np.concatenate((left_points_3D, np.ones((1, left_points_3D.shape[1]))))
    back_p3d = np.concatenate((back_points_3D, np.ones((1, back_points_3D.shape[1]))))

    # Get the 3D points in the world
    front_p3d_world = np.dot(front_extrinsic_matrix, front_p3d)[:3]
    right_p3d_world = np.dot(right_extrinsic_matrix, right_p3d)[:3]
    left_p3d_world = np.dot(left_extrinsic_matrix, left_p3d)[:3]
    back_p3d_world = np.dot(back_extrinsic_matrix, back_p3d)[:3]

    # Reshape the array to (height * width, 3) -> X, Y and Z for each point
    front_p3d_world = np.transpose(front_p3d_world)
    right_p3d_world = np.transpose(right_p3d_world)
    left_p3d_world = np.transpose(left_p3d_world)
    back_p3d_world = np.transpose(back_p3d_world)
    
    points = np.concatenate((front_p3d_world, right_p3d_world, left_p3d_world, back_p3d_world))
    colors = np.concatenate((front_color, right_color, left_color, back_color))
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(points, axis=0)
    points = points - centroid
    
    print(f"PointCloud with {front_p3d_world.shape[0] + right_p3d_world.shape[0] + left_p3d_world.shape[0] + back_p3d_world.shape[0]} points")

    return points, colors, front_extrinsic_matrix



def main():
    actor_list = []
    pcl_downsampled = o3d.geometry.PointCloud()
    cc = carla.ColorConverter.LogarithmicDepth

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        
        settings = world.get_settings()
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)

        # "DayClear" | "DayCloudy" | "DayRain" | "NigthCloudy"
        environment.weather_environment("DayClear", world)
        
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)        
        traffic_manager.ignore_lights_percentage(vehicle, 100)  # Ignore all the red ligths

    # Spawn cameras to get the data to the Dataset
        camera_rgb = spawn_sensor.spawn_sensores('sensor.camera.rgb', world, blueprint_library, vehicle, camera_attributes)
        camera_depth = spawn_sensor.spawn_sensores('sensor.camera.depth', world, blueprint_library, vehicle, camera_attributes)
        camera_lidar = spawn_sensor.spawn_sensores('sensor.lidar.ray_cast', world, blueprint_library, vehicle, lidar_attributes)
        print("Sensors spawned!")
        
    # Spawn cameras to get the ground truth
        front_depth_camera, front_rgb_camera, right_depth_camera, left_depth_camera, back_depth_camera = ground_truth.spawn_cameras(world, blueprint_library, vehicle, 1280, 960)
        print("Ground Truth cameras spawned!")
        
        # Add the actors to the list
        actor_list.extend([vehicle, camera_rgb, camera_depth, camera_lidar, front_depth_camera, front_rgb_camera, right_depth_camera, left_depth_camera, back_depth_camera])

    
    # Queues to get the data
        image_queue_rgb = queue.Queue()
        image_queue_depth = queue.Queue()
        image_queue_lidar = queue.Queue()
        # Ground Truth
        image_queue_rgb_front = queue.Queue()
        image_queue_depth_front = queue.Queue()
        image_queue_depth_right = queue.Queue()
        image_queue_depth_left = queue.Queue()
        image_queue_depth_back = queue.Queue()
        

    # Listen to the cameras
        camera_rgb.listen(image_queue_rgb.put)
        camera_depth.listen(image_queue_depth.put)
        camera_lidar.listen(image_queue_lidar.put)
        # Ground Truth
        front_rgb_camera.listen(image_queue_rgb_front.put)
        front_depth_camera.listen(image_queue_depth_front.put)
        right_depth_camera.listen(image_queue_depth_right.put)
        left_depth_camera.listen(image_queue_depth_left.put)
        back_depth_camera.listen(image_queue_depth_back.put)
        
        #flag = True
        while cv2.waitKey(1) != ord('q'):
        #while flag:
            world.tick()
            #flag = False
        # GROUND TRUTH
            queue_list = {"image_queue_rgb_front": image_queue_rgb_front, "image_queue_depth_front": image_queue_depth_front,
                          "image_queue_depth_right": image_queue_depth_right, "image_queue_depth_left": image_queue_depth_left, 
                          "image_queue_depth_back": image_queue_depth_back}
            
            depth_camera_list = {"front_depth_camera": front_depth_camera, "right_depth_camera": right_depth_camera, 
                                 "left_depth_camera": left_depth_camera, "back_depth_camera": back_depth_camera}
            
            points, colors, extrinsic = get_ground_truth(queue_list, depth_camera_list)
        

        # DOWNSAMPLING
            downsampled_points, downsampled_colors= ground_truth.downsample(points, colors, args.leaf_size)
            
            pcl_downsampled.points = o3d.utility.Vector3dVector(downsampled_points)
            pcl_downsampled.colors = o3d.utility.Vector3dVector(downsampled_colors)
            
            
        # GET THE CENTER OF THE CAMERAS                    
            red_indices = np.where(colors[:, 0] == 255)[0]
            red_center_points = points[red_indices]

            # Get the center of the red points (Coords of the cameras)
            groundtruth_center = np.mean(red_center_points, axis=0)
            # Add the center of the cameras to the point cloud
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.points), groundtruth_center]))
            pcl_downsampled.colors = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.colors), [255, 0, 0]]))


        # LIDAR TRANSFORMATION
            lidar_data = image_queue_lidar.get()
            raw_data = lidar_data.raw_data
            lidar_pcl, center_lidar = lidar_transformation(extrinsic, raw_data)



            """ # Get the (0,0,0) - black color points
            ground_truth_black_indices = np.where(np.asarray(pcl_downsampled.colors)[:,] == [0,0,0])[0] """

        # Fit the lidar point cloud to the ground truth point cloud
            translation_to_center = center_lidar - groundtruth_center
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.array(pcl_downsampled.points) + translation_to_center)
            
                    # DELETE THE RED POINTS ->>>>>>>>>>>>>>>>>> FIX THIS <<<<<<<<<<<<<<<<<<<<
            ground_truth_red_indices = np.where(np.asarray(pcl_downsampled.colors)[:, 0] == 255)[0]
            ground_truth_points = np.delete(np.asarray(pcl_downsampled.points), ground_truth_red_indices, axis=0)
            ground_truth_colors = np.delete(np.asarray(pcl_downsampled.colors), ground_truth_red_indices, axis=0)
            pcl_downsampled.points = o3d.utility.Vector3dVector(ground_truth_points)
            ground_truth_colors = np.clip(ground_truth_colors / 255, 0, 1)
            pcl_downsampled.colors = o3d.utility.Vector3dVector(ground_truth_colors)
            
            lidar_red_indices = np.where(np.asarray(lidar_pcl.colors)[:, 0] == 255)[0]
            lidar_points = np.delete(np.asarray(lidar_pcl.points), lidar_red_indices, axis=0)
            lidar_colors = np.delete(np.asarray(lidar_pcl.colors), lidar_red_indices, axis=0)
            lidar_pcl.points = o3d.utility.Vector3dVector(lidar_points)
            lidar_colors = np.clip(lidar_colors / 255, 0, 1)
            lidar_pcl.colors = o3d.utility.Vector3dVector(lidar_colors)

            
            
            

    # SAVE THE DATA
        # Save the RGB image
            image = image_queue_rgb.get()
            image.save_to_disk('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png')
        # Save the Depth image
            image = image_queue_depth.get()
            image.save_to_disk('_out/depth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png', cc)
        # Save the Lidar point cloud
            o3d.io.write_point_cloud(f'./_out/lidar/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.ply', lidar_pcl)
        # Save the Ground Truth point cloud
            o3d.io.write_point_cloud(f'./_out/ground_truth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.ply', pcl_downsampled)
            
            
            
            #o3d.visualization.draw_geometries([pcl_downsampled, lidar_pcl])
        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()