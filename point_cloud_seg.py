from setup import setup_world
from spawn import spawn_vehicle, spawn_sensor
import queue

lidar_attributes = {
    
    "real_lidar": {
        'channels': '128',                      # Numero de lasers
        'range': '75.0',                        # Distancia máxima em metros
        'points_per_second': '2621440',         # Pontos por segundo
        'rotation_frequency': '20',             # Velocidade de rotação Hz
        'upper_fov': '45',                      # Qual o ângulo do centro para cima
        'lower_fov': '-45',                     # Qual o ângulo do centro para baixo
    },
}


def main():
    actor_list = []

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        settings = world.get_settings()
        
        settings.fixed_delta_seconds = 0.05
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
        sensor_lidar_segm = spawn_sensor.spawn_sensores('sensor.lidar.ray_cast_semantic', world, blueprint_library, vehicle, lidar_attributes)
        actor_list.append(sensor_lidar_segm)
        print(f"Camera Lidar: {sensor_lidar_segm}")

    
        image_queue_lidar_segm = queue.Queue()

        sensor_lidar_segm.listen(image_queue_lidar_segm.put)


        while True:
            world.tick()
            
            image = image_queue_lidar_segm.get()
            
            #print(f"Image: {image}")
            #for detection in image:
                #print(f"{detection.point} | Object_tag: {detection.object_tag}")
            
            image.save_to_disk('_out/lidarSegm/%06d' % image.frame + '.ply')

        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")




if __name__ == '__main__':
    main()