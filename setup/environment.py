import carla


def weather_environment(world, precip_deposits, cloudiness, precipitation, 
                        sun_angle, fog, wetness, wind, air_pollution, dust_storm):
    
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
        
    weather = carla.WeatherParameters(
        precipitation_deposits = precip_deposits,
        cloudiness= cloudiness,
        precipitation= precipitation,
        sun_altitude_angle= sun_angle,
        fog_density = fog,
        wetness = wetness,
        wind_intensity = wind,
        mie_scattering_scale = air_pollution, 
        dust_storm = dust_storm)

    world.set_weather(weather)
