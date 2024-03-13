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
        
        #settings = world.get_settings()
        #settings.fixed_delta_seconds = 2
        #world.apply_settings(settings)

        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        actor_list.append(vehicle)
        print(f"Vehicle: {vehicle}")
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        
        environment.weather_environment(world, 
                                        precip_deposits=10,
                                        cloudiness=5,
                                        precipitation=10,
                                        sun_angle=10,
                                        fog=0,
                                        wetness=5,
                                        wind=5,
                                        air_pollution=0,
                                        dust_storm=0)
        
        camera_rgb = spawn_sensor.spawn_camera('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT)
        actor_list.append(camera_rgb)
        print(f"Camera RGB: {camera_rgb}")

        camera_depth = spawn_sensor.spawn_camera('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT)
        actor_list.append(camera_depth)
        print(f"Camera Depth: {camera_depth}")
        
        camera_lidar = spawn_sensor.spawn_sensor('sensor.lidar.ray_cast', world, blueprint_library, vehicle)
        actor_list.append(camera_lidar)
        print(f"Camera Lidar: {camera_lidar}")
        
        
        
        camera_lidar.listen(lambda image: image.save_to_disk('_out/lidar/%06d.ply' % image.frame))
        
        camera_rgb.listen(lambda image: image.save_to_disk('_out/rgb/%06d.png' % image.frame))
    
        cc = carla.ColorConverter.LogarithmicDepth
        camera_depth.listen(lambda image: image.save_to_disk('_out/depth/%06d.png' % image.frame, cc))
        
        
        
        time.sleep(1)

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()