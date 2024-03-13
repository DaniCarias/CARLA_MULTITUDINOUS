import random
import carla

# Atributos da camera RGB o mais parecidos com a camera real
def set_atributes_rgb(camera_bp):
    camera_bp.set_attribute('fov', f"{110}") # Field of view horizontal em graus 
    # Opening of the camera lens (fstop) -> ver se é o 2.1 / 4.0 
    camera_bp.set_attribute('enable_postprocess_effects', 'True') # Efeito de realismo na imagem 


# Atributos da camera de Depth o mais parecidos com a camera real
def set_atributes_depth(camera_bp):
    camera_bp.set_attribute('fov', f"{120}") # Field of view horizontal em graus


def spawn_camera(camera, world, blueprint_library, vehicle, img_width, img_height):
    camera_bp = blueprint_library.find(camera)
    
    if camera == 'sensor.camera.rgb':
        set_atributes_rgb(camera_bp)
        
    if camera == 'sensor.camera.depth':
        set_atributes_depth(camera_bp)
    
    # Resolução da imagem de output
    camera_bp.set_attribute('image_size_x', f"{img_width}")
    camera_bp.set_attribute('image_size_y', f"{img_height}")
    camera_bp.set_attribute('sensor_tick', f"{0.4}") # Frequencia de captura de imagem
    
    camera_transform = carla.Transform(carla.Location(x=0.9, z=1.3)) # Posição da camera em metros
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

def spawn_sensor(sensor, world, blueprint_library, vehicle):
    sensor_bp = blueprint_library.find(sensor)
    sensor_bp.set_attribute('sensor_tick', f"{0.4}") # Frequencia de captura de imagem
    
    sensor_bp.set_attribute('channels', str(128))                # Numero de lasers
    sensor_bp.set_attribute('range', str(75.0))                  # Distancia máxima em metros
    sensor_bp.set_attribute('points_per_second', str(2621440))   # Pontos por segundo
    sensor_bp.set_attribute('rotation_frequency', str(20))       # Velocidade de rotação
    sensor_bp.set_attribute('upper_fov', str(45))                # Qual o ângulo do centro para cima
    sensor_bp.set_attribute('lower_fov', str(-45))               # Qual o ângulo do centro para baixo
    sensor_bp.set_attribute('dropoff_general_rate', str(0.1))    # Qual a percentagem de pontos que vai descartar (0.1 tal como a Lidar que vamos usar) -> Colocar a 0 para voxel grid
    sensor_bp.set_attribute('dropoff_intensity_limit', str(0.8)) # O que for a baixo do valor de intensidade é descartado -> Colocar a 0 para voxel grid
    sensor_bp.set_attribute('dropoff_zero_intensity', str(0.4))  # Se o valor de intensidade for 0 qual a percentagem de descaratar -> Colocar a 1 para voxel grid
    
    sensor_transform = carla.Transform(carla.Location(x=0, z=2.4)) # Posição do sensor em metros
    sensor = world.spawn_actor(sensor_bp, sensor_transform, attach_to=vehicle)
    return sensor