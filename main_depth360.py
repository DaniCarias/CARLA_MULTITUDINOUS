import time
from setup import setup_world, environment
from spawn import spawn_vehicle
import carla
import queue
import math


def spawn_cameras(camera, world, blueprint_library, vehicle, img_width, img_height, camera_transform):
    camera_bp = blueprint_library.find(camera)
    
    camera_bp.set_attribute('image_size_x', f"{img_width}")
    camera_bp.set_attribute('image_size_y', f"{img_height}")
    
    camera_bp.set_attribute('fov', f"{120}")
    camera_bp.set_attribute('lens_k', f"{0}") # Remove distortion
    camera_bp.set_attribute('lens_kcube', f"{0}") # Remove distortion
        
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

def main():
    actor_list = []
    IMG_WIDTH = 1280
    IMG_HEIGHT = 720

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        settings = world.get_settings()
        
        settings.fixed_delta_seconds = 1
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
        world.apply_settings(settings)

        
        # "DayClear" | "DayCloudy" | "DayRain" | "NigthCloudy"
        environment.weather_environment("DayClear", world)
    
    # Vehicle
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        actor_list.append(vehicle)
        print(f"Vehicle: {vehicle}")
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        
        
    # Depth 1
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5))
        camera_depth1 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        actor_list.append(camera_depth1)
        print(f"Camera Depth1: {camera_depth1}")

    # Depth 2
        rotation120 = carla.Rotation(yaw=120.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation120)
        camera_depth2 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        actor_list.append(camera_depth2)
        print(f"Camera Depth2: {camera_depth2}")
        
    # Depth 3
        rotation240 = carla.Rotation(yaw=240.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation240)
        camera_depth3 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        actor_list.append(camera_depth3)
        print(f"Camera Depth3: {camera_depth3}")
        
        
        
        image_queue_depth1 = queue.Queue()
        image_queue_depth2 = queue.Queue()
        image_queue_depth3 = queue.Queue()
        

        camera_depth1.listen(image_queue_depth1.put)
        camera_depth2.listen(image_queue_depth2.put)
        camera_depth3.listen(image_queue_depth3.put)

        while True:
            world.tick()
            cc = carla.ColorConverter.LogarithmicDepth
            
            image = image_queue_depth1.get()
            image.save_to_disk('_out/ground_truth/depth1/%06d' % image.frame + '.png', cc)
            
            image = image_queue_depth2.get()
            image.save_to_disk('_out/ground_truth/depth2/%06d' % image.frame + '.png', cc)

            image = image_queue_depth3.get()
            image.save_to_disk('_out/ground_truth/depth3/%06d' % image.frame + '.png', cc)
        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")




if __name__ == '__main__':
    main()