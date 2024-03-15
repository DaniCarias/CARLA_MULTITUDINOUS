import glob
import os
import sys
import carla

def setup_carla():
    try:
        sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
            sys.version_info.major,
            sys.version_info.minor,
            'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    except IndexError:
        pass

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    """
    if not world.get_map().name == 'Carla/Maps/Town01_Opt':
        print(f"Current world: {world.get_map().name}")
        client.load_world('Town01_Opt')
    """
        
    # To edit the traffic 
    traffic_manager = client.get_trafficmanager(8000)

    blueprint_library = world.get_blueprint_library()

    return world, blueprint_library, traffic_manager