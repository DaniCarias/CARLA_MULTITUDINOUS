import time
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
from utils.ground_truth import ground_truth_right_hand as ground_truth
import carla
import queue
import cv2
import numpy as np
import open3d as o3d
import math as mt
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

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


def lidar_transformation(extrinsic, raw_data):
    lidar_pcl = o3d.geometry.PointCloud()
    
    point_cloud_array = np.frombuffer(raw_data, dtype=np.float32)
    point_cloud_array = np.reshape(point_cloud_array, (int(point_cloud_array.shape[0] / 4), 4))            
    point_cloud_array = np.delete(point_cloud_array, 3, 1)
    
    
    
# Fix the lidar point cloud transformation to world coordinates
     # yaw = 90
    yaw_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    point_cloud_array = np.dot(point_cloud_array, yaw_90)
    # z = -z
    point_cloud_array[:, 2] = -point_cloud_array[:, 2]

    
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
    rotation_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(rotation_z, center=(0, 0, 0)) 
    
    
    # Right hand system
    array = np.array(lidar_pcl.points)
    array[:,1] = -array[:,1] # Invert the Y axis
    lidar_pcl.points = o3d.utility.Vector3dVector(array)
    

    # Rotation of (Extract rotation around Z-axis of the ground truth camera) in the Z axis
    """ theta_z = np.arctan2(extrinsic[1, 0], extrinsic[0, 0])
    rotation_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(rotation_z, center=(0, 0, 0)) """
    
    
# TESTE
    # add point 0,0,0
    point_cloud_array = np.vstack([np.array(lidar_pcl.points), [0, 0, 0]])
    mask_points = np.array(lidar_pcl.points)
    mask_colors = np.zeros((mask_points.shape[0], 3))
    mask_points = np.vstack([mask_points, [0, 0, 0]])
    mask_colors = np.vstack([mask_colors, [255, 0, 0]])
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(mask_points), axis=0)
    mask_points = mask_points - centroid
    
    colors = np.zeros((np.array(lidar_pcl.points).shape[0], 3))
    colors = np.vstack([colors, [255,0,0]])
    lidar_pcl.colors = o3d.utility.Vector3dVector(colors)
    
    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(lidar_pcl.points), axis=0)
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
# MEIO TESTE
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
    

    points = np.concatenate((front_p3d_world, right_p3d_world, left_p3d_world, back_p3d_world))
    colors = np.concatenate((front_color, right_color, left_color, back_color))
    
    
# TESTE
    points_mask = np.concatenate((points_mask1_word, points_mask2_word, points_mask3_word, points_mask4_word))
    colors_mask = np.concatenate((colors_mask1, colors_mask2, colors_mask3, colors_mask4))
    
    
    
# RIGHT HAND SYSTEM
    points[:,1] = -points[:,1]           # Invert the Y axis
    points_mask[:,1] = -points_mask[:,1] # Invert the Y axis
    
    
    
    
    # MASK - CENTER
    centroid = np.mean(points_mask, axis=0)
    points_mask = points_mask - centroid
    
    # ORIGINAL - CENTER
    centroid = np.mean(points, axis=0)
    points = points - centroid

    
    #print(f"PointCloud with {front_p3d_world.shape[0] + right_p3d_world.shape[0] + left_p3d_world.shape[0] + back_p3d_world.shape[0]} points")

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
        # GET THE POINTS WITH RED COLOR
            red_center_points = np.zeros((0, 3))
            id = 0
            for point in colors_mask:
                if point[0] == 255:
                    point_coord = np.array(points_mask[id])
                    red_center_points = np.vstack([red_center_points, point_coord]) # Get the origin of each point cloud (90º)
                id += 1
                
        # ADD THE CENTER OF THE GROUND TRUTH
            groundtruth_center = np.mean(red_center_points, axis=0)
            print(f"Ground Truth center: {groundtruth_center}")
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.points), groundtruth_center]))
            pcl_downsampled.colors = o3d.utility.Vector3dVector(np.vstack([np.array(pcl_downsampled.colors), [255, 0, 0]]))
            
            
            
            

            
            
            
    # LIDAR TRANSFORMATION
            lidar_data = image_queue_lidar.get()
            raw_data = lidar_data.raw_data
            lidar_pcl, center_lidar = lidar_transformation(extrinsic, raw_data)
            print(f"Lidar center: {center_lidar}")











# TESTE
            trans = center_lidar - groundtruth_center
            pcl_downsampled.points = o3d.utility.Vector3dVector(np.array(pcl_downsampled.points) + trans)
            
            




    # SAVE THE DATA
        # Save the RGB image
            original_image = image_queue_rgb.get()
            #original_image.save_to_disk('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % original_image.frame + '.png')

            
            raw_data = np.frombuffer(original_image.raw_data, dtype=np.uint8)
            img_rgb = np.reshape(raw_data, (original_image.height, original_image.width, 4))
            img_rgb = img_rgb[:, :, :3]
            #img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
            #original_image = Image.fromarray(img_rgb)
            
            
            flipped_image = cv2.flip(img_rgb, 1)
            #flipped_image = Image.fromarray(flipped_image)
            #img.save('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % original_image.frame + 'rgb_fliped.png.png')
            
            
            # visualize the image with cv2
            while cv2.waitKey(1) != ord('q'):
                cv2.imshow('RGB Camera Output', img_rgb)
            while cv2.waitKey(1) != ord('q'):
                cv2.imshow('RGB Camera Output FLIPED ', flipped_image)
            
            
            
            
            
            #image.save_to_disk('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '_flipped.png')
            
            
            
        # Save the Depth image
            image = image_queue_depth.get()
            """ flipped_image = cv2.flip(image, 1)
            image.save_to_disk('_out/depth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png', cc)
            flipped_image.save_to_disk('_out/depth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '_flipped.png', cc) """
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