from utils.setup import setup_world, environment
from utils.spawn import spawn_vehicle
from utils.ground_truth import ground_truth_with_colors
import carla
import queue
import numpy as np
import open3d as o3d

camera_attributes = {
    "real_rgb": {
        'fov': '110',                    # Field of view horizontal in degrees 110 -> Based on Zed 2i camera
        'fstop': '0.5',                  # Aperture -> 1/2 -> Based on Zed 2i camera
        'iso': '1900',                   # Sensitivity -> Based on Zed 2i camera
        'focal_distance': '2000',        # Distance to the focal point     
        'motion_blur_intensity': '0.1',  # Intensity of the motion blur
    },
        
    "rgb_and_depth": {
        'image_size_x': '1280',
        'image_size_y': '720',
        'fov': '110',                  # Field of view horizontal in degrees 110 -> Based on Zed 2i camera
        'lens_k': '0',                 # Remove the distortion
        'lens_kcube': '0',             # Remove the distortion
    },
}


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

    front_intrinsic_matrix, front_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['front_depth_camera'], front_depth_image)
    right_intrinsic_matrix, right_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['right_depth_camera'], right_depth_image)
    left_intrinsic_matrix, left_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['left_depth_camera'], left_depth_image)
    back_intrinsic_matrix, back_extrinsic_matrix = ground_truth_with_colors.get_intrinsic_extrinsic_matrix(depth_camera_list['back_depth_camera'], back_depth_image)

    # Get the points [[X...], [Y...], [Z...]] and the colors [[R...], [G...], [B...]] normalized
    front_points_3D, front_color = ground_truth_with_colors.point2D_to_point3D(front_depth_image, front_rbg_image[..., [2, 1, 0]], front_intrinsic_matrix)
    right_points_3D, right_color = ground_truth_with_colors.point2D_to_point3D(right_depth_image, right_rbg_image[..., [2, 1, 0]], right_intrinsic_matrix)
    left_points_3D,  left_color = ground_truth_with_colors.point2D_to_point3D(left_depth_image, left_rbg_image[..., [2, 1, 0]], left_intrinsic_matrix)
    back_points_3D,  back_color = ground_truth_with_colors.point2D_to_point3D(back_depth_image, back_rbg_image[..., [2, 1, 0]], back_intrinsic_matrix)

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
    
    #print(f"PointCloud with {front_p3d_world.shape[0] + right_p3d_world.shape[0] + left_p3d_world.shape[0] + back_p3d_world.shape[0]} points")

    pcl_360 = o3d.geometry.PointCloud()
    pcl_360.points = o3d.utility.Vector3dVector(points)
    pcl_360.colors = o3d.utility.Vector3dVector(np.clip(colors/255, 0, 1))

    return points, colors



def main():
    actor_list = []
    pcl_downsampled = o3d.geometry.PointCloud()

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla("Town01_Opt")
        
        settings = world.get_settings()
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)

        # "DayClear" | "DayCloudy" | "DayRain" | "NigthCloudy"
        environment.weather_environment("DayClear", world)
        
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)        
        traffic_manager.ignore_lights_percentage(vehicle, 100)  # Ignore all the red ligths

        
    # Spawn cameras to get the ground truth
        front_depth_camera, front_rgb_camera, right_depth_camera, right_rgb_camera, left_depth_camera, left_rgb_camera, back_depth_camera, back_rgb_camera = ground_truth_with_colors.spawn_cameras(world, blueprint_library, vehicle, 1280, 960)
        print("Ground Truth cameras spawned!")
        
        # Add the actors to the list
        actor_list.extend([vehicle, front_depth_camera, front_rgb_camera, right_depth_camera, right_rgb_camera, left_depth_camera, left_rgb_camera, back_depth_camera, back_rgb_camera])

    
    # Queues
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
        # Ground Truth
        front_rgb_camera.listen(image_queue_rgb_front.put)
        front_depth_camera.listen(image_queue_depth_front.put)
        right_rgb_camera.listen(image_queue_rgb_right.put)
        right_depth_camera.listen(image_queue_depth_right.put)
        left_rgb_camera.listen(image_queue_rgb_left.put)
        left_depth_camera.listen(image_queue_depth_left.put)
        back_rgb_camera.listen(image_queue_rgb_back.put)
        back_depth_camera.listen(image_queue_depth_back.put)
        
        while True:
            world.tick()
    
        # GROUND TRUTH
            queue_list = {"image_queue_rgb_front": image_queue_rgb_front, "image_queue_depth_front": image_queue_depth_front,
                          "image_queue_rgb_right": image_queue_rgb_right, "image_queue_depth_right": image_queue_depth_right, 
                          "image_queue_rgb_left": image_queue_rgb_left, "image_queue_depth_left": image_queue_depth_left, 
                          "image_queue_rgb_back": image_queue_rgb_back, "image_queue_depth_back": image_queue_depth_back}
            depth_camera_list = {"front_depth_camera": front_depth_camera, "right_depth_camera": right_depth_camera, 
                                 "left_depth_camera": left_depth_camera, "back_depth_camera": back_depth_camera}
            points, colors = get_ground_truth(queue_list, depth_camera_list)


    # DOWNSAMPLING
            downsampled_points, downsampled_colors= ground_truth_with_colors.downsample(points, colors, 0.2) #arguments.leaf_size
            downsampled_colors = np.clip(downsampled_colors / 255.0, 0, 1)

            pcl_downsampled.points = o3d.utility.Vector3dVector(downsampled_points)
            pcl_downsampled.colors = o3d.utility.Vector3dVector(downsampled_colors)

            o3d.visualization.draw_geometries([pcl_downsampled])


    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()