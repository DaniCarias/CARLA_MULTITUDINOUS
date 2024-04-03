import carla


"""
    Weather Parameters:
    
    - precip_deposits  -> Poças de água [0, 100]
    - cloudiness       -> Nuvens [0, 100]
    - precipitation    -> Precipitação [0, 100]
    - sun_angle        -> Ângulo da altura do sol [-90, 90]
    - fog              -> Densidade do nevoeiro [0, 100] -> Afeta a visibilidade da camera RGB
    - wetness          -> Humidade [0, 100] -> Afeta a visibilidade da camera RGB
    - wind             -> Intensidade do vento [0, 100] -> Muda a direção da chuva e das folhas das árvores
    - air_pollution    -> Poluição no ar [0, 1]
    - dust_storm       -> Tempestade de areia [0, 100]
"""

weather_types = {
    "DayClear":{
        'precip_deposits': 10,
        'cloudiness': 5,
        'precipitation': 10,
        'sun_angle': 10,
        'fog': 0,
        'wetness': 5,
        'wind': 5,
        'air_pollution': 0,
        'dust_storm': 0
    },
    "DayCloudy":{
        'precip_deposits': 10,
        'cloudiness': 50,
        'precipitation': 10,
        'sun_angle': 10,
        'fog': 0,
        'wetness': 5,
        'wind': 5,
        'air_pollution': 0,
        'dust_storm': 0
    },
    "DayRain":{
        'precip_deposits': 40,
        'cloudiness': 60,
        'precipitation': 100,
        'sun_angle': 40,
        'fog': 0,
        'wetness': 100,
        'wind': 100,
        'air_pollution': 0,
        'dust_storm': 0
    },
    "NigthCloudy":{
        'precip_deposits': 10,
        'cloudiness': 20,
        'precipitation': 10,
        'sun_angle': -10,
        'fog': 0,
        'wetness': 5,
        'wind': 5,
        'air_pollution': 0,
        'dust_storm': 0
    }
}


def weather_environment(weather_type, world):
    
    attrs = weather_types.get(weather_type, weather_types[weather_type])
    
    weather = carla.WeatherParameters(
        precipitation_deposits = attrs['precip_deposits'],
        cloudiness= attrs['cloudiness'],
        precipitation= attrs['precipitation'],
        sun_altitude_angle= attrs['sun_angle'],
        fog_density = attrs['fog'],
        wetness = attrs['wetness'],
        wind_intensity = attrs['wind'],
        mie_scattering_scale = attrs['air_pollution'], 
        dust_storm = attrs['dust_storm']
    )
        
    world.set_weather(weather)  
    