from setup import setup_world
from spawn import spawn_vehicle
import carla
import queue
import numpy as np
import math as mt
import open3d as o3d
from numpy.matlib import repmat
import cv2
import keyboard

def spawn_cameras(camera, world, blueprint_library, vehicle, img_width, img_height, camera_transform):
    camera_bp = blueprint_library.find(camera)
    
    camera_bp.set_attribute('image_size_x', f"{img_width}")
    camera_bp.set_attribute('image_size_y', f"{img_height}")
    
    camera_bp.set_attribute('fov', f"{90}")
    camera_bp.set_attribute('lens_k', f"{0}")
    camera_bp.set_attribute('lens_kcube', f"{0}")
    camera_bp.set_attribute('lens_circle_falloff', f"{0}")

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera


def _to_bgra_array(image):
    """Convert a CARLA raw image to a BGRA numpy array."""
    if not isinstance(image, carla.Image):
        raise ValueError("Argument must be a carla.sensor.Image")
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))

    return array

def _depth_to_array(image):
    """
    Convert an image containing CARLA encoded depth-map to a 2D array containing
    the depth value of each pixel normalized between [0.0, 1.0].
    """
    array = _to_bgra_array(image)
    array = array.astype(np.float32)
    # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    normalized_depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
    normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

    return normalized_depth


def get_intrinsic_extrinsic_matrix(camera_depth, image_depth):
    
# Extrinsic matrix
    #transform_matrix = camera_depth.get_transform().get_matrix()
    camera2vehicle_matrix = np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=np.float64)
    
    pitch = camera_depth.get_transform().rotation.pitch / 180.0 * mt.pi
    yaw = camera_depth.get_transform().rotation.yaw / 180.0 * mt.pi
    roll = camera_depth.get_transform().rotation.roll / 180.0 * mt.pi
    loc_x = camera_depth.get_transform().location.x
    loc_y = - camera_depth.get_transform().location.y
    loc_z = camera_depth.get_transform().location.z
    sin_y, sin_p, sin_r = mt.sin(yaw), mt.sin(pitch), mt.sin(roll)
    cos_y, cos_p, cos_r = mt.cos(yaw), mt.cos(pitch), mt.cos(roll)

    transform_matrix = np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r + sin_y * cos_r, - cos_y * sin_p * cos_r + sin_y * sin_r, loc_x],
        [-sin_y * cos_p, - sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r + cos_y * sin_r, loc_y],
        [sin_p, -cos_p * sin_r, cos_p * cos_r, loc_z],
        [0.0, 0.0, 0.0, 1.0]
    ])

    camera2world_matrix = transform_matrix @ camera2vehicle_matrix

# Intrinsics matrix
    focal_lengthX = image_depth.width / (2 * mt.tan(90 * mt.pi / 360.0))
    centerX = image_depth.width / 2
    centerY = image_depth.height / 2

    intrinsic_matrix = [[focal_lengthX, 0, centerX],
                        [0, focal_lengthX, centerY],
                        [0, 0, 1]]
    
    return intrinsic_matrix, camera2world_matrix


def point2D_to_point3D(image_depth, image_rgb, intrinsic_matrix):
    
    intrinsic_matrix_inv = np.linalg.inv(intrinsic_matrix)
    
    pixel_length = image_depth.width * image_depth.height
    
    # Return a array (height, width, 4) with the BGRA values of each pixel
    normalized_depth = _depth_to_array(image_depth)
    normalized_depth = np.reshape(normalized_depth, pixel_length)

    color = image_rgb.reshape(pixel_length, 3)
    
    u_coord = repmat(np.r_[image_depth.width-1:-1:-1],
                     image_depth.height, 1).reshape(pixel_length)
    v_coord = repmat(np.c_[image_depth.height-1:-1:-1],
                     1, image_depth.width).reshape(pixel_length)
    
    
    # Delete the pixels with depth > 0.9
    max_depth_indexes = np.where(normalized_depth > 0.9)
    
    normalized_depth = np.delete(normalized_depth, max_depth_indexes)
    u_coord = np.delete(u_coord, max_depth_indexes)
    v_coord = np.delete(v_coord, max_depth_indexes)
    color = np.delete(color, max_depth_indexes, axis=0)
    
    depth_in_meters = normalized_depth*1000
    
    # Convert the 2D pixel coordinates to 3D points
    p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])
    p3d = np.dot(intrinsic_matrix_inv, p2d) * depth_in_meters

    # Return [[X...], [Y...], [Z...]]
    return p3d, color


def main():
    point_cloud = o3d.geometry.PointCloud()
    actor_list = []
    IMG_WIDTH = 1280 # 800
    IMG_HEIGHT = 720 # 600

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        
    # Settings
        settings = world.get_settings()
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

            
    # Vehicle
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        vehicle.set_autopilot(True)
        traffic_manager.ignore_lights_percentage(vehicle, 100) # Ignore all the red ligths
        
        
    # Depth & RGB FRONT
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5))
        camera_depth_front = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb_front = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
    # Depth & RGB RIGHT
        camera_transform_right = carla.Transform(carla.Location(x=0.9, z=2.5), carla.Rotation(yaw=90.0))
        camera_depth_right = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_right)
        camera_rgb_right = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_right)
    # Depth & RGB LEFT
        camera_transform_left = carla.Transform(carla.Location(x=0.9, z=2.5), carla.Rotation(yaw=-90.0))
        camera_depth_left = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_left)
        camera_rgb_left = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_left)
    # Depth & RGB BACK
        camera_transform_back = carla.Transform(carla.Location(x=0, z=2.5), carla.Rotation(yaw=180.0))
        camera_depth_back = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_back)
        camera_rgb_back = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform_back)
        
        actor_list.extend([vehicle, camera_depth_front, camera_rgb_front, camera_depth_right, camera_rgb_right, camera_depth_left, camera_rgb_left, camera_depth_back, camera_rgb_back])

    # Queues
        image_queue_depth_front = queue.Queue()
        image_queue_rgb_front = queue.Queue()
        image_queue_depth_right = queue.Queue()
        image_queue_rgb_right = queue.Queue()
        image_queue_depth_left = queue.Queue()
        image_queue_rgb_left = queue.Queue()
        image_queue_depth_back = queue.Queue()
        image_queue_rgb_back = queue.Queue()
        
    # Listen to the cameras
        camera_depth_front.listen(image_queue_depth_front.put)
        camera_rgb_front.listen(image_queue_rgb_front.put)
        camera_depth_right.listen(image_queue_depth_right.put)
        camera_rgb_right.listen(image_queue_rgb_right.put)
        camera_depth_left.listen(image_queue_depth_left.put)
        camera_rgb_left.listen(image_queue_rgb_left.put)
        camera_depth_back.listen(image_queue_depth_back.put)
        camera_rgb_back.listen(image_queue_rgb_back.put)
        
        tick = -1
        while cv2.waitKey(1) != ord('q'):
            
            world.tick()
            tick += 1
            
            image_depth_front = image_queue_depth_front.get()
            rbg_image_front = image_queue_rgb_front.get()
            image_depth_right = image_queue_depth_right.get()
            rbg_image_right = image_queue_rgb_right.get()
            image_depth_left = image_queue_depth_left.get()
            rbg_image_left = image_queue_rgb_left.get()
            image_depth_back = image_queue_depth_back.get()
            rbg_image_back = image_queue_rgb_back.get()
            
        # Get the data with a interval of 30 ticks
            if tick % 30 == 0:
                
                rbg_image_front = np.reshape(np.copy(rbg_image_front.raw_data), (rbg_image_front.height, rbg_image_front.width, 4))
                rbg_image_right = np.reshape(np.copy(rbg_image_right.raw_data), (rbg_image_right.height, rbg_image_right.width, 4))
                rbg_image_left = np.reshape(np.copy(rbg_image_left.raw_data), (rbg_image_left.height, rbg_image_left.width, 4))
                rbg_image_back = np.reshape(np.copy(rbg_image_back.raw_data), (rbg_image_back.height, rbg_image_back.width, 4))
                
                cv2.imshow('RGB Camera Front Output', rbg_image_front)
                #cv2.imshow('RGB Camera Right Output', rbg_image_right)
                #cv2.imshow('RGB Camera Left Output', rbg_image_left)
                cv2.imshow('RGB Camera Back Output', rbg_image_back)
                
                intrinsic_matrix_front, camera2world_matrix_front = get_intrinsic_extrinsic_matrix(camera_depth_front, image_depth_front)
                intrinsic_matrix_right, camera2world_matrix_right = get_intrinsic_extrinsic_matrix(camera_depth_right, image_depth_right)
                intrinsic_matrix_left, camera2world_matrix_left = get_intrinsic_extrinsic_matrix(camera_depth_left, image_depth_left)
                intrinsic_matrix_back, camera2world_matrix_back = get_intrinsic_extrinsic_matrix(camera_depth_back, image_depth_back)
                
                
            # Get the 3D points [[X...], [Y...], [Z...]]
                points_3D_front, color_front = point2D_to_point3D(image_depth_front, rbg_image_front[..., [2, 1, 0]], intrinsic_matrix_front)
                points_3D_right, color_right = point2D_to_point3D(image_depth_right, rbg_image_right[..., [2, 1, 0]], intrinsic_matrix_right)
                points_3D_left, color_left = point2D_to_point3D(image_depth_left, rbg_image_left[..., [2, 1, 0]], intrinsic_matrix_left)
                points_3D_back, color_back = point2D_to_point3D(image_depth_back, rbg_image_back[..., [2, 1, 0]], intrinsic_matrix_back)
                
                
            # To multiply by the extrinsic matrix (same shape as the camera2world_matrix matrix)
                p3d_front = np.concatenate((points_3D_front, np.ones((1, points_3D_front.shape[1]))))
                p3d_right = np.concatenate((points_3D_right, np.ones((1, points_3D_right.shape[1]))))
                p3d_left = np.concatenate((points_3D_left, np.ones((1, points_3D_left.shape[1]))))
                p3d_back = np.concatenate((points_3D_back, np.ones((1, points_3D_back.shape[1]))))
                
            # Get the 3D points in the world
                p3d_world_front = np.dot(camera2world_matrix_front, p3d_front)[:3]
                p3d_world_right = np.dot(camera2world_matrix_right, p3d_right)[:3]
                p3d_world_left = np.dot(camera2world_matrix_left, p3d_left)[:3]
                p3d_world_back = np.dot(camera2world_matrix_back, p3d_back)[:3]
                
            # Reshape the array to (height * width, 3) -> X, Y and Z for each point
                p3d_world_front = np.transpose(p3d_world_front)
                p3d_world_right = np.transpose(p3d_world_right)
                p3d_world_left = np.transpose(p3d_world_left)
                p3d_world_back = np.transpose(p3d_world_back)
                
            # Add the points and colors to the point cloud
                point_cloud.points.extend(o3d.utility.Vector3dVector(p3d_world_front))
                point_cloud.colors.extend(o3d.utility.Vector3dVector(color_front / 255.0))
                point_cloud.points.extend(o3d.utility.Vector3dVector(p3d_world_right))
                point_cloud.colors.extend(o3d.utility.Vector3dVector(color_right / 255.0))
                point_cloud.points.extend(o3d.utility.Vector3dVector(p3d_world_left))
                point_cloud.colors.extend(o3d.utility.Vector3dVector(color_left / 255.0))
                point_cloud.points.extend(o3d.utility.Vector3dVector(p3d_world_back))
                point_cloud.colors.extend(o3d.utility.Vector3dVector(color_back / 255.0))
                
                print(f"{point_cloud}")

            if tick!=0 and tick % 120 == 0:
                o3d.visualization.draw_geometries([point_cloud])
            
    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")


if __name__ == '__main__':
    main()