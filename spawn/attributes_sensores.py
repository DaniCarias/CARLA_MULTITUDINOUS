attributes = {
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
        'noise_stddev': '0.1',                  # Desvio padrão do ruído
    },
}

def set_attributes_lidar(sensor_bp):
    
    attrs = attributes.get("real_lidar", attributes["real_lidar"])

    for attr, value in attrs.items():
        print(f"LIDAR: {attr}: {value}")
        sensor_bp.set_attribute(str(attr), str(value))
    
    
# Atributos da camera RGB o mais parecidos com a camera real
def set_atributes_rgb(camera_bp):
    camera_bp.set_attribute('fov', f"{110}") # Field of view horizontal em graus 110 -> Baseado na camera Zed   
    camera_bp.set_attribute('fstop', f"{0.5}") # Abertura da lente -> 1/2 -> Baseado na camera Zed
    camera_bp.set_attribute('iso', f"{1900}") # Sensitivity do sensor -> Baseado na camera Zed
    camera_bp.set_attribute('focal_distance', f"{2000}") # Distância a que a profundidade do efeito de campo deve ser nítida.     
    camera_bp.set_attribute('motion_blur_intensity', f"{0}") # Intensidade do efeito de movimento
    
    # focal_length = 2.12   |   # focal_center = IMG_WIDTH(1280)/2 and IMG_HEIGHT(720)/2
    camera_bp.set_attribute('lens_k', f"{0}")
    camera_bp.set_attribute('lens_kcube', f"{0}")



