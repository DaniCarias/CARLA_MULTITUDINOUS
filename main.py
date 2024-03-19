import time
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
import carla
import queue


def main():
    actor_list = []
    IMG_WIDTH = 1280
    IMG_HEIGHT = 720

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        
        # No rendering mode
        settings = world.get_settings()
        settings.no_rendering_mode = True
        # Put synchronous mode
        settings.fixed_delta_seconds = 0.5
        settings.synchronous_mode = True # Enables synchronous mode
        world.apply_settings(settings)



        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        actor_list.append(vehicle)
        print(f"Vehicle: {vehicle}")
        
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        
        # "DayClear" | "DayCloudy" | "DayRain" | "NigthCloudy"
        environment.weather_environment("DayClear", world)
        
        
        
        camera_rgb = spawn_sensor.spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT)
        actor_list.append(camera_rgb)
        print(f"Camera RGB: {camera_rgb}")
        
        camera_depth = spawn_sensor.spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT)
        actor_list.append(camera_depth)
        print(f"Camera Depth: {camera_depth}")
        
        # "voxel_grid" | "real_lidar"
        camera_lidar = spawn_sensor.spawn_lidar('sensor.lidar.ray_cast', world, blueprint_library, vehicle, "real_lidar")
        actor_list.append(camera_lidar)
        print(f"Camera Lidar: {camera_lidar}")
        
        
        
        image_queue_rgb = queue.Queue()
        image_queue_depth = queue.Queue()
        image_queue_lidar = queue.Queue()
        
        camera_rgb.listen(image_queue_rgb.put)
        camera_depth.listen(image_queue_depth.put)
        camera_lidar.listen(image_queue_lidar.put)


        while True:
            world.tick()
            image = image_queue_rgb.get()
            image.save_to_disk('_out/rgb/%06d' % image.frame + f"_{time.strftime('%d%m%Y_%H%M%S')}.png")
            
            image = image_queue_depth.get()
            cc = carla.ColorConverter.LogarithmicDepth
            image.save_to_disk('_out/depth/%06d' % image.frame + f"_{time.strftime('%d%m%Y_%H%M%S')}.png", cc)

            image = image_queue_lidar.get()
            image.save_to_disk('_out/lidar/%06d' % image.frame + f"_{time.strftime('%d%m%Y_%H%M%S')}.ply")
        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()