import time
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
from utils.ground_truth import ground_truth
import carla
import queue
import cv2
import numpy as np
import open3d as o3d
import math as mt

lidar_attributes = {
    # Lidar sem distorção:
    #'dropoff_general_rate': '0',
    #'dropoff_intensity_limit': '0',
    #'dropoff_zero_intensity': '0',
    #'atmosphere_attenuation_rate': '0',
    #'noise_stddev': '0'
    
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
        'motion_blur_intensity': '0',  # Intensidade do efeito de movimento
    },
        
    "rgb_and_depth": {
        'image_size_x': '1280',
        'image_size_y': '720',
        'fov': '110',                  # Field of view horizontal em graus 110 -> Baseado na camera Zed
        'lens_k': '0',                 # Remove the distortion
        'lens_kcube': '0',             # Remove the distortion
    },
}



def lidar_transformation2(raw_data, lidar, extrinsic):
    lidar_pcl = o3d.geometry.PointCloud()
    
    
    scale_x = 1
    scale_y = 1
    scale_z = -1
    rotation = lidar.get_transform().rotation
    location = lidar.get_transform().location
    c_y = np.cos(np.radians(rotation.yaw + 90)) #-270
    s_y = np.sin(np.radians(rotation.yaw + 90)) #-270
    c_r = np.cos(np.radians(0))
    s_r = np.sin(np.radians(0))
    c_p = np.cos(np.radians(0))
    s_p = np.sin(np.radians(0))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = scale_x * (c_p * c_y)
    matrix[0, 1] = scale_x * (c_y * s_p * s_r + s_y * c_r)
    matrix[0, 2] = scale_x * (-c_y * s_p * c_r + s_y * s_r)
    matrix[1, 0] = scale_y * (-s_y * c_p)
    matrix[1, 1] = scale_y * (-s_y * s_p * s_r + c_y * c_r)
    matrix[1, 2] = scale_y * (s_y * s_p * c_r + c_y * s_r)
    matrix[2, 0] = scale_z * (s_p)
    matrix[2, 1] = scale_z * (-c_p * s_r)
    matrix[2, 2] = scale_z * (c_p * c_r)
    
    
    point_cloud = np.frombuffer(raw_data, dtype=np.float32)
    point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 4), 4))            
    
    point_cloud = point_cloud @ matrix
    
    point_cloud = np.delete(point_cloud, 3, 1)
    
    # z = -z
    point_cloud[:, 2] = -point_cloud[:, 2]
    # change the x and y
    point_cloud[:, [0, 1]] = point_cloud[:, [1, 0]]
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(point_cloud, axis=0)
    # Translate the point cloud to the origin
    point_cloud = point_cloud - centroid
    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud)
    
   
    # Z rotation of the camera
    # Extract rotation around Z-axis
    theta_z = np.arctan2(extrinsic[1, 0], extrinsic[0, 0])
    rotation__z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(rotation__z, center=(0, 0, 0))
    
    
    return lidar_pcl

def lidar_transformation(extrinsic, raw_data, lidar):
    lidar_pcl = o3d.geometry.PointCloud()
    
    point_cloud_array = np.frombuffer(raw_data, dtype=np.float32)
    point_cloud_array = np.reshape(point_cloud_array, (int(point_cloud_array.shape[0] / 4), 4))            
    point_cloud_array = np.delete(point_cloud_array, 3, 1)
    
    
    
# Fix the lidar point cloud transformation
    # yaw = 90
    array = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    point_cloud_array = point_cloud_array @ array
    # z = -z
    point_cloud_array[:, 2] = -point_cloud_array[:, 2]

    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
# Transform the point cloud to the camera coordinate system
    # Rotation of 180 in the x axis
    x_180 = np.array([[1,  0,  0],
                      [0, -1,  0],
                      [0,  0, -1]])
    lidar_pcl.rotate(x_180, center=(0, 0, 0))
    
    # Rotation of 180 in the y axis
    y_180 = np.array([[-1,  0,  0],
                      [ 0, -1,  0],
                      [ 0,  0,  1]])
    lidar_pcl.rotate(y_180, center=(0, 0, 0))
    
    # Z rotation of the camera
    # Extract rotation around Z-axis
    theta_z = np.arctan2(extrinsic[1, 0], extrinsic[0, 0])
    rotation__z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(rotation__z, center=(0, 0, 0))
    
# TESTE
    # add point 0,0,0
    point_cloud_array = np.vstack([np.array(lidar_pcl.points), [0, 0, 0]])
    mask_points = np.array(lidar_pcl.points)
    mask_colors = np.zeros((mask_points.shape[0], 3))
    mask_points = np.vstack([mask_points, [0, 0, 0]])
    mask_colors = np.vstack([mask_colors, [255, 0, 0]])
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(mask_points), axis=0)
    # Translate the point cloud to the origin
    mask_points = mask_points - centroid
    
    colors = np.zeros((np.array(lidar_pcl.points).shape[0], 3))
    colors = np.vstack([colors, [255,0,0]])
    lidar_pcl.colors = o3d.utility.Vector3dVector(colors)
    
    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(lidar_pcl.points), axis=0)
    # Translate the point cloud to the origin
    point_cloud_array = np.array(lidar_pcl.points) - centroid
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)  
    
    
# TESTE
    id = 0
    for point in mask_colors:
        if point[0] == 255:
            center_lidar = mask_points[id]
        id += 1
    
    
    return lidar_pcl, center_lidar
        
        
# To get the ground truth
def get_ground_truth(queue_list, depth_camera_list):
    
    front_depth_image = queue_list['image_queue_depth_front'].get()
    front_rbg_image = queue_list['image_queue_rgb_front'].get()
    right_depth_image = queue_list['image_queue_depth_right'].get()
    right_rbg_image = queue_list['image_queue_rgb_right'].get()
    left_depth_image = queue_list['image_queue_depth_left'].get()
    left_rbg_image = queue_list['image_queue_rgb_left'].get()
    back_depth_image = queue_list['image_queue_depth_back'].get()
    back_rbg_image = queue_list['image_queue_rgb_back'].get()
    
    front_rbg_image = np.reshape(np.copy(front_rbg_image.raw_data), (front_rbg_image.height, front_rbg_image.width, 4))
    right_rbg_image = np.reshape(np.copy(right_rbg_image.raw_data), (right_rbg_image.height, right_rbg_image.width, 4))
    left_rbg_image = np.reshape(np.copy(left_rbg_image.raw_data), (left_rbg_image.height, left_rbg_image.width, 4))
    back_rbg_image = np.reshape(np.copy(back_rbg_image.raw_data), (back_rbg_image.height, back_rbg_image.width, 4))

    #cv2.imshow('RGB Camera Front Output', front_rbg_image)

    front_intrinsic_matrix, front_camera2world_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['front_depth_camera'], front_depth_image)
    right_intrinsic_matrix, right_camera2world_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['right_depth_camera'], right_depth_image)
    left_intrinsic_matrix, left_camera2world_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['left_depth_camera'], left_depth_image)
    back_intrinsic_matrix, back_camera2world_matrix = ground_truth.get_intrinsic_extrinsic_matrix(depth_camera_list['back_depth_camera'], back_depth_image)

    # Get the points [[X...], [Y...], [Z...]] and the colors [[R...], [G...], [B...]] normalized
    front_points_3D, front_color, points_mask1, colors_mask1 = ground_truth.point2D_to_point3D(front_depth_image, front_rbg_image[..., [2, 1, 0]], front_intrinsic_matrix)
    right_points_3D, right_color, points_mask2, colors_mask2 = ground_truth.point2D_to_point3D(right_depth_image, right_rbg_image[..., [2, 1, 0]], right_intrinsic_matrix)
    left_points_3D, left_color, points_mask3, colors_mask3 = ground_truth.point2D_to_point3D(left_depth_image, left_rbg_image[..., [2, 1, 0]], left_intrinsic_matrix)
    back_points_3D, back_color, points_mask4, colors_mask4 = ground_truth.point2D_to_point3D(back_depth_image, back_rbg_image[..., [2, 1, 0]], back_intrinsic_matrix)

    # To multiply by the extrinsic matrix (same shape as the camera2world_matrix matrix)
    front_p3d = np.concatenate((front_points_3D, np.ones((1, front_points_3D.shape[1]))))
    right_p3d = np.concatenate((right_points_3D, np.ones((1, right_points_3D.shape[1]))))
    left_p3d = np.concatenate((left_points_3D, np.ones((1, left_points_3D.shape[1]))))
    back_p3d = np.concatenate((back_points_3D, np.ones((1, back_points_3D.shape[1]))))
    
    
# TESTE
    points_mask1 = np.concatenate((points_mask1, np.ones((1, points_mask1.shape[1]))))
    points_mask2 = np.concatenate((points_mask2, np.ones((1, points_mask2.shape[1]))))
    points_mask3 = np.concatenate((points_mask3, np.ones((1, points_mask3.shape[1]))))
    points_mask4 = np.concatenate((points_mask4, np.ones((1, points_mask4.shape[1]))))
    
    

    # Get the 3D points in the world
    front_p3d_world = np.dot(front_camera2world_matrix, front_p3d)[:3]
    right_p3d_world = np.dot(right_camera2world_matrix, right_p3d)[:3]
    left_p3d_world = np.dot(left_camera2world_matrix, left_p3d)[:3]
    back_p3d_world = np.dot(back_camera2world_matrix, back_p3d)[:3]
    
    
# TESTE
    points_mask1_word = np.dot(front_camera2world_matrix, points_mask1)[:3]
    points_mask2_word = np.dot(right_camera2world_matrix, points_mask2)[:3]
    points_mask3_word = np.dot(left_camera2world_matrix, points_mask3)[:3]
    points_mask4_word = np.dot(back_camera2world_matrix, points_mask4)[:3]
    
    
    
 
    # Reshape the array to (height * width, 3) -> X, Y and Z for each point
    front_p3d_world = np.transpose(front_p3d_world)
    right_p3d_world = np.transpose(right_p3d_world)
    left_p3d_world = np.transpose(left_p3d_world)
    back_p3d_world = np.transpose(back_p3d_world)
    
    
# TESTE
    points_mask1_word = np.transpose(points_mask1_word)
    points_mask2_word = np.transpose(points_mask2_word)
    points_mask3_word = np.transpose(points_mask3_word)
    points_mask4_word = np.transpose(points_mask4_word)
    colors_mask1 = np.transpose(colors_mask1)
    colors_mask2 = np.transpose(colors_mask2)
    colors_mask3 = np.transpose(colors_mask3)
    colors_mask4 = np.transpose(colors_mask4)
    

    #colors = np.zeros((0, 3))
    #points = np.zeros((0, 3))

    points = np.concatenate((front_p3d_world, right_p3d_world, left_p3d_world, back_p3d_world))
    colors = np.concatenate((front_color, right_color, left_color, back_color))
    
    
# TESTE
    points_mask = np.concatenate((points_mask1_word, points_mask2_word, points_mask3_word, points_mask4_word))
    colors_mask = np.concatenate((colors_mask1, colors_mask2, colors_mask3, colors_mask4))
    centroid = np.mean(points_mask, axis=0)
    points_mask = points_mask - centroid
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(points, axis=0)
    # Translate the point cloud to the origin
    points = points - centroid
    
    print(f"PointCloud with {front_p3d_world.shape[0] + right_p3d_world.shape[0] + left_p3d_world.shape[0] + back_p3d_world.shape[0]} points")

    return points, colors, front_camera2world_matrix, points_mask, colors_mask



def main():
    actor_list = []
    pcl_downsampled = o3d.geometry.PointCloud()
    cc = carla.ColorConverter.LogarithmicDepth

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        
        settings = world.get_settings()
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
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
        front_depth_camera, front_rgb_camera, right_depth_camera, right_rgb_camera, left_depth_camera, left_rgb_camera, back_depth_camera, back_rgb_camera = ground_truth.spawn_cameras(world, blueprint_library, vehicle, 1280, 960)
        print("Ground Truth cameras spawned!")
        
        # Add the actors to the list
        actor_list.extend([vehicle, camera_rgb, camera_depth, camera_lidar, front_depth_camera, front_rgb_camera, right_depth_camera, right_rgb_camera, left_depth_camera, left_rgb_camera, back_depth_camera, back_rgb_camera])

    
    # Queues
        # Dataset
        image_queue_rgb = queue.Queue()
        image_queue_depth = queue.Queue()
        image_queue_lidar = queue.Queue()
        # Ground Truth
        image_queue_rgb_front = queue.Queue()
        image_queue_depth_front = queue.Queue()
        image_queue_rgb_right = queue.Queue()
        image_queue_depth_right = queue.Queue()
        image_queue_rgb_left = queue.Queue()
        image_queue_depth_left = queue.Queue()
        image_queue_rgb_back = queue.Queue()
        image_queue_depth_back = queue.Queue()
        

    # Listen to the cameras
        # Dataset
        camera_rgb.listen(image_queue_rgb.put)
        camera_depth.listen(image_queue_depth.put)
        camera_lidar.listen(image_queue_lidar.put)
        # Ground Truth
        front_rgb_camera.listen(image_queue_rgb_front.put)
        front_depth_camera.listen(image_queue_depth_front.put)
        right_rgb_camera.listen(image_queue_rgb_right.put)
        right_depth_camera.listen(image_queue_depth_right.put)
        left_rgb_camera.listen(image_queue_rgb_left.put)
        left_depth_camera.listen(image_queue_depth_left.put)
        back_rgb_camera.listen(image_queue_rgb_back.put)
        back_depth_camera.listen(image_queue_depth_back.put)
        
        flag = True
        #while cv2.waitKey(1) != ord('q'):
        while flag:
            world.tick()
    
            flag = False
    # GROUND TRUTH
            queue_list = {"image_queue_rgb_front": image_queue_rgb_front, "image_queue_depth_front": image_queue_depth_front,
                          "image_queue_rgb_right": image_queue_rgb_right, "image_queue_depth_right": image_queue_depth_right, 
                          "image_queue_rgb_left": image_queue_rgb_left, "image_queue_depth_left": image_queue_depth_left, 
                          "image_queue_rgb_back": image_queue_rgb_back, "image_queue_depth_back": image_queue_depth_back}
            depth_camera_list = {"front_depth_camera": front_depth_camera, "right_depth_camera": right_depth_camera, 
                                 "left_depth_camera": left_depth_camera, "back_depth_camera": back_depth_camera}
            points, colors, extrinsic, points_mask, colors_mask = get_ground_truth(queue_list, depth_camera_list)
        

    # DOWNSAMPLING
            downsampled_points, downsampled_colors= ground_truth.downsample(points, colors, 0.1) #arguments.leaf_size
            downsampled_colors = np.clip(downsampled_colors / 255.0, 0, 1)

            pcl_downsampled.points = o3d.utility.Vector3dVector(downsampled_points)
            pcl_downsampled.colors = o3d.utility.Vector3dVector(downsampled_colors)
            
            
# TESTE
            # Get the points with the red color
            red_center_points = np.zeros((0, 3))
            id = 0
            for point in colors_mask:
                if point[0] == 255:
                    point_coord = np.array(points_mask[id])
                    red_center_points = np.vstack([red_center_points, point_coord])   # verificar se estam so os 4 pontos 
                id += 1
                
            groundtruth_center = np.mean(red_center_points, axis=0)
            print(f"Ground Truth center: {groundtruth_center}")
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.points), groundtruth_center]))
            pcl_downsampled.colors = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.colors), [255, 0, 0]]))
            
            
            
            

            
            
            
    # LIDAR TRANSFORMATION
            lidar_data = image_queue_lidar.get()
            raw_data = lidar_data.raw_data
            lidar_pcl, center_lidar = lidar_transformation(extrinsic, raw_data, camera_lidar)
            #lidar_pcl = lidar_transformation2(raw_data, front_depth_camera, extrinsic)
            print(f"Lidar center: {center_lidar}")





# TESTE
            trans = center_lidar - groundtruth_center
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.array(pcl_downsampled.points) + trans)
            #lidar_pcl.points = o3d.utility.Vector3dVector(np.array(lidar_pcl.points) - center_lidar)
            







    # SAVE THE DATA
        # Save the RGB image
            image = image_queue_rgb.get()
            #image.save_to_disk('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png')
        # Save the Depth image
            image = image_queue_depth.get()
            #image.save_to_disk('_out/depth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png', cc)
        # Save the Lidar point cloud
            #o3d.io.write_point_cloud(f'./_out/lidar/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.ply', lidar_pcl)
        # Save the Ground Truth point cloud
            #o3d.io.write_point_cloud(f'./_out/ground_truth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.ply', pcl_downsampled)
            
            
            o3d.visualization.draw_geometries([pcl_downsampled, lidar_pcl])
        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()