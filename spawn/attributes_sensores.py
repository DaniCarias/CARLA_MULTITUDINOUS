


attributes = {
    "real_lidar": {
        'channels': '128',                  # Numero de lasers
        'range': '75.0',                    # Distancia máxima em metros -> 75.0
        'points_per_second': '2621440',     # Pontos por segundo
        'rotation_frequency': '20',         # Velocidade de rotação
        'upper_fov': '45',                  # Qual o ângulo do centro para cima
        'lower_fov': '-45',                 # Qual o ângulo do centro para baixo
        'dropoff_general_rate': '0.1',      # Qual a percentagem de pontos que vai descartar
        'dropoff_intensity_limit': '0.8',   # O que for a baixo do valor de intensidade é descartado 
        'dropoff_zero_intensity': '0.4'     # Se o valor de intensidade for 0 qual a percentagem de descaratar -> Colocar a 1 para voxel grid
    },
    "voxel_grid": {
        'channels': '256',
        'range': '75.0',
        'points_per_second': '10485760',
        'rotation_frequency': '20', 
        'upper_fov': '60',
        'lower_fov': '-60',
        'dropoff_general_rate': '0',
        'dropoff_intensity_limit': '0',
        'dropoff_zero_intensity': '0'
    }
}

def set_attributes_lidar(sensor_bp, attribute_type):
    
    attrs = attributes.get(attribute_type, attributes[attribute_type])

    for attr, value in attrs.items():
        sensor_bp.set_attribute(str(attr), str(value))
    
    
# Atributos da camera RGB o mais parecidos com a camera real
def set_atributes_rgb(camera_bp):
    camera_bp.set_attribute('fov', f"{110}") # Field of view horizontal em graus 
    camera_bp.set_attribute('enable_postprocess_effects', 'True') # Efeito de realismo na imagem   
    
    camera_bp.set_attribute('fstop', '1.4')
     # Opening of the camera lens (fstop) -> ver se é o 2.1 / 4.0 


# Atributos da camera de Depth o mais parecidos com a camera real
def set_atributes_depth(camera_bp):
    camera_bp.set_attribute('fov', f"{120}") # Field of view horizontal em graus


