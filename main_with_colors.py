import time
from utils.setup import setup_world, environment
from utils.spawn import spawn_sensor, spawn_vehicle
from utils.ground_truth import ground_truth_with_colors
import carla
import queue
import cv2
import numpy as np
import open3d as o3d
import math as mt
from PIL import Image

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
    yaw_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])   # yaw = 90
    point_cloud_array = np.dot(point_cloud_array, yaw_90)
    
    point_cloud_array[:, 2] = -point_cloud_array[:, 2]      # z = -z


    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
# Transform the point cloud to the camera coordinate system
    x_180 = np.array([[1,  0,  0],
                      [0, -1,  0],
                      [0,  0, -1]])
    lidar_pcl.rotate(x_180, center=(0, 0, 0)) # Rotation of 180 in the X axis
    
    y_180 = np.array([[-1,  0,  0],
                      [ 0, -1,  0],
                      [ 0,  0,  1]])
    lidar_pcl.rotate(y_180, center=(0, 0, 0)) # Rotation of 180 in the Y axis
    
    # Rotation of (Extract rotation around Z-axis of the ground truth camera) in the Z axis
    theta_z = np.arctan2(extrinsic[1, 0], extrinsic[0, 0])
    rotation_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z),  0],
        [0,               0,                1]
    ])
    lidar_pcl.rotate(rotation_z, center=(0, 0, 0))
    
# TESTE
    # Add point 0,0,0
    point_cloud_array = np.vstack([np.array(lidar_pcl.points), [0, 0, 0]])
    mask_points = np.array(lidar_pcl.points)
    mask_colors = np.zeros((mask_points.shape[0], 3))
    mask_points = np.vstack([mask_points, [0, 0, 0]])
    mask_colors = np.vstack([mask_colors, [255, 0, 0]])
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(mask_points), axis=0)
    mask_points = mask_points - centroid
    
    # Add the red point to the point cloud
    colors = np.zeros((np.array(lidar_pcl.points).shape[0], 3))
    colors = np.vstack([colors, [255,0,0]])
    lidar_pcl.colors = o3d.utility.Vector3dVector(colors)
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(np.array(lidar_pcl.points), axis=0)
    point_cloud_array = np.array(lidar_pcl.points) - centroid
    
    lidar_pcl.points = o3d.utility.Vector3dVector(point_cloud_array)  
    
    
    id = 0
    for point in mask_colors:
        if point[0] == 255:
            center_lidar = mask_points[id]
        id += 1
    
    # VERIFICAR SE O CENTRO DO LIDAR É O (0,0,0), SE FOR N É PRECISO O CODIGO A CIMA
    
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

    cv2.imshow('RGB Camera Front Output', front_rbg_image) # Show the image

    front_intrinsic_matrix, front_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['front_depth_camera'], front_depth_image)
    right_intrinsic_matrix, right_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['right_depth_camera'], right_depth_image)
    left_intrinsic_matrix, left_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['left_depth_camera'], left_depth_image)
    back_intrinsic_matrix, back_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['back_depth_camera'], back_depth_image)

    # Get the points [[X...], [Y...], [Z...]] and the colors [[R...], [G...], [B...]] normalized
    # Get the mask of the points and the colors to get the center of the point cloud
    front_points_3D, front_color, points_mask_front, colors_mask_front = ground_truth_with_colors.point2D_to_point3D(front_depth_image, front_rbg_image[..., [2, 1, 0]], front_intrinsic_matrix)
    right_points_3D, right_color, points_mask_right, colors_mask_right = ground_truth_with_colors.point2D_to_point3D(right_depth_image, right_rbg_image[..., [2, 1, 0]], right_intrinsic_matrix)
    left_points_3D,  left_color,  points_mask_left,  colors_mask_left = ground_truth_with_colors.point2D_to_point3D(left_depth_image, left_rbg_image[..., [2, 1, 0]], left_intrinsic_matrix)
    back_points_3D,  back_color,  points_mask_back,  colors_mask_back = ground_truth_with_colors.point2D_to_point3D(back_depth_image, back_rbg_image[..., [2, 1, 0]], back_intrinsic_matrix)

    # To multiply by the extrinsic matrix (same shape as the extrinsic_matrix matrix)
    front_p3d = np.concatenate((front_points_3D, np.ones((1, front_points_3D.shape[1]))))
    right_p3d = np.concatenate((right_points_3D, np.ones((1, right_points_3D.shape[1]))))
    left_p3d = np.concatenate((left_points_3D, np.ones((1, left_points_3D.shape[1]))))
    back_p3d = np.concatenate((back_points_3D, np.ones((1, back_points_3D.shape[1]))))
    
    
# TESTE
    points_mask_front = np.concatenate((points_mask_front, np.ones((1, points_mask_front.shape[1]))))
    points_mask_right = np.concatenate((points_mask_right, np.ones((1, points_mask_right.shape[1]))))
    points_mask_left = np.concatenate((points_mask_left, np.ones((1, points_mask_left.shape[1]))))
    points_mask_back = np.concatenate((points_mask_back, np.ones((1, points_mask_back.shape[1]))))
    
    

    # Get the 3D points in the world
    front_p3d_world = np.dot(front_extrinsic_matrix, front_p3d)[:3]
    right_p3d_world = np.dot(right_extrinsic_matrix, right_p3d)[:3]
    left_p3d_world = np.dot(left_extrinsic_matrix, left_p3d)[:3]
    back_p3d_world = np.dot(back_extrinsic_matrix, back_p3d)[:3]
    
    
# TESTE
    points_mask_front_word = np.dot(front_extrinsic_matrix, points_mask_front)[:3]
    points_mask_right_word = np.dot(right_extrinsic_matrix, points_mask_right)[:3]
    points_mask_left_word = np.dot(left_extrinsic_matrix, points_mask_left)[:3]
    points_mask_back_word = np.dot(back_extrinsic_matrix, points_mask_back)[:3]
    
    
    
 
    # Reshape the array to (height * width, 3) -> X, Y and Z for each point
    front_p3d_world = np.transpose(front_p3d_world)
    right_p3d_world = np.transpose(right_p3d_world)
    left_p3d_world = np.transpose(left_p3d_world)
    back_p3d_world = np.transpose(back_p3d_world)
    
    
# TESTE
    points_mask_front_word = np.transpose(points_mask_front_word)
    points_mask_right_word = np.transpose(points_mask_right_word)
    points_mask_left_word = np.transpose(points_mask_left_word)
    points_mask_back_word = np.transpose(points_mask_back_word)
    colors_mask_front = np.transpose(colors_mask_front)
    colors_mask_right = np.transpose(colors_mask_right)
    colors_mask_left = np.transpose(colors_mask_left)
    colors_mask_back = np.transpose(colors_mask_back)
    
    pcl_90 = o3d.geometry.PointCloud()
    pcl_90.points = o3d.utility.Vector3dVector(front_p3d_world)
    pcl_90.colors = o3d.utility.Vector3dVector(np.clip(front_color/255, 0, 1))
    o3d.visualization.draw_geometries([pcl_90])
    

    points = np.concatenate((front_p3d_world, right_p3d_world, left_p3d_world, back_p3d_world))
    colors = np.concatenate((front_color, right_color, left_color, back_color))
    
    
# TESTE
    points_mask = np.concatenate((points_mask_front_word, points_mask_right_word, points_mask_left_word, points_mask_back_word))
    colors_mask = np.concatenate((colors_mask_front, colors_mask_right, colors_mask_left, colors_mask_back))
    centroid = np.mean(points_mask, axis=0)
    points_mask = points_mask - centroid
    
    
    # Put the center of the point cloud in the origin
    centroid = np.mean(points, axis=0)
    points = points - centroid
    
    print(f"PointCloud with {front_p3d_world.shape[0] + right_p3d_world.shape[0] + left_p3d_world.shape[0] + back_p3d_world.shape[0]} points")


    pcl_360 = o3d.geometry.PointCloud()
    pcl_360.points = o3d.utility.Vector3dVector(points)
    pcl_360.colors = o3d.utility.Vector3dVector(np.clip(colors/255, 0, 1))
    o3d.visualization.draw_geometries([pcl_360])


    return points, colors, front_extrinsic_matrix, points_mask, colors_mask



def main():
    actor_list = []
    pcl_downsampled = o3d.geometry.PointCloud()
    cc = carla.ColorConverter.LogarithmicDepth

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla("Town02_Opt")
        
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
        front_depth_camera, front_rgb_camera, right_depth_camera, right_rgb_camera, left_depth_camera, left_rgb_camera, back_depth_camera, back_rgb_camera = ground_truth_with_colors.spawn_cameras(world, blueprint_library, vehicle, 1280, 960)
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
            downsampled_points, downsampled_colors= ground_truth_with_colors.downsample(points, colors, 0.2) #arguments.leaf_size
            downsampled_colors = np.clip(downsampled_colors / 255.0, 0, 1)

            pcl_downsampled.points = o3d.utility.Vector3dVector(downsampled_points)
            pcl_downsampled.colors = o3d.utility.Vector3dVector(downsampled_colors)
            
            o3d.visualization.draw_geometries([pcl_downsampled])

            
            
# TESTE
            # Get the points with the red color
            red_center_points = np.zeros((0, 3))
            id = 0
            for point in colors_mask:
                if point[0] == 255:
                    point_coord = np.array(points_mask[id])
                    red_center_points = np.vstack([red_center_points, point_coord]) # Get the origin of each point cloud (90º)
                id += 1
                
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
            
            

            # Delete the points with the red color
            ground_truth_red_indices = np.where(np.asarray(pcl_downsampled.colors)[:, 0] == 255)[0]
            ground_truth_points = np.delete(np.asarray(pcl_downsampled.points), ground_truth_red_indices, axis=0)
            ground_truth_colors = np.delete(np.asarray(pcl_downsampled.colors), ground_truth_red_indices, axis=0)
            pcl_downsampled.points = o3d.utility.Vector3dVector(ground_truth_points)
            #ground_truth_colors = np.clip(ground_truth_colors / 255, 0, 1)
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
            
            raw_data = np.frombuffer(image.raw_data, dtype=np.uint8)
            img_rgb = np.reshape(raw_data, (image.height, image.width, 4))
            img_rgb = img_rgb[:, :, :3]
            img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
            flipped_image = cv2.flip(img_rgb, 1)
            #image.raw_data = img_rgb.tobytes() # Não dá para dar set!!!!
            
            # Save the flipped_image
            img = Image.fromarray(flipped_image)
            img.save('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + 'rgb_fliped.png.png')
            
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
            
            
            o3d.visualization.draw_geometries([pcl_downsampled]) #lidar_pcl


    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()