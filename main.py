import time
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
import carla

def main():
    actor_list = []
    IMG_WIDTH = 1280
    IMG_HEIGHT = 720

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        
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
        camera_lidar = spawn_sensor.spawn_lidar('sensor.lidar.ray_cast', world, blueprint_library, vehicle, "voxel_grid")
        actor_list.append(camera_lidar)
        print(f"Camera Lidar: {camera_lidar}")
        
        
        camera_lidar.listen(lambda image: image.save_to_disk('_out/lidar/%06d.ply' % image.frame))
        
        camera_rgb.listen(lambda image: image.save_to_disk('_out/rgb/%06d.png' % image.frame))
    
        cc = carla.ColorConverter.LogarithmicDepth
        camera_depth.listen(lambda image: image.save_to_disk('_out/depth/%06d.png' % image.frame, cc))
        
        
        time.sleep(8)

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()