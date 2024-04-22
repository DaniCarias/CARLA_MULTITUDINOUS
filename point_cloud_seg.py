import time
from setup import setup_world, environment
from spawn import spawn_vehicle
from spawn import spawn_lidar
import carla
import queue
import math

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

            
    # Vehicle
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        actor_list.append(vehicle)
        print(f"Vehicle: {vehicle}")
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        
        
    # Lidar Segmentation
        camera_lidar = spawn.spawn_lidar('sensor.lidar.ray_cast', world, blueprint_library, vehicle)
        actor_list.append(camera_lidar)
        print(f"Camera Lidar: {camera_lidar}")

    

        image_queue_depth1 = queue.Queue()

        

        camera_depth1.listen(image_queue_depth1.put)


        while True:
            world.tick()
            
            image = image_queue_depth1.get()
            image.save_to_disk('_out/ground_truth/degree_0/depth/%06d' % image.frame + '.png')

        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")




if __name__ == '__main__':
    main()