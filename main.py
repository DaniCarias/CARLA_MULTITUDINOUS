import time
from setup import setup_world, environment
from spawn import spawn_sensor, spawn_vehicle
import carla
import queue


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
        #'noise_stddev': '0.1',                  # Ruído
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

def main():
    actor_list = []


    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        settings = world.get_settings()
        
        settings.fixed_delta_seconds = 1
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
        world.apply_settings(settings)

        
        # "DayClear" | "DayCloudy" | "DayRain" | "NigthCloudy"
        environment.weather_environment("DayClear", world)
        
        
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        actor_list.append(vehicle)
        print(f"Vehicle: {vehicle}")
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)

        
        camera_rgb = spawn_sensor.spawn_sensores('sensor.camera.rgb', world, blueprint_library, vehicle, camera_attributes)
        actor_list.append(camera_rgb)
        print(f"Camera RGB: {camera_rgb}")
        
        camera_depth = spawn_sensor.spawn_sensores('sensor.camera.depth', world, blueprint_library, vehicle,camera_attributes)
        actor_list.append(camera_depth)
        print(f"Camera Depth: {camera_depth}")
        
        camera_lidar = spawn_sensor.spawn_sensores('sensor.lidar.ray_cast', world, blueprint_library, vehicle, lidar_attributes)
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
            image.save_to_disk('_out/rgb/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png')
            
            image = image_queue_depth.get()
            cc = carla.ColorConverter.LogarithmicDepth
            image.save_to_disk('_out/depth/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.png', cc)
            
            image = image_queue_lidar.get()
            image.save_to_disk('_out/lidar/' + time.strftime('%Y%m%d_%H%M%S') + '_%06d' % image.frame + '.ply') 
        

    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")

if __name__ == '__main__':
    main()