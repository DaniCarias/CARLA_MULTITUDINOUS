import carla
from utils.setup import setup_world, environment


def main():
    actor_list = []
    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
                    
        settings = world.get_settings()
        settings.no_rendering_mode = False # No rendering mode
        settings.synchronous_mode = True  # Enables synchronous mode
        settings.fixed_delta_seconds = 0.02
        world.apply_settings(settings)
                        
        spawn_points = world.get_map().get_spawn_points()

        environment.weather_environment("DayClear", world)


        # Print all the spawn points of the map
        for i, spawn_point in enumerate(spawn_points):
            world.debug.draw_string(spawn_point.location, str(i), life_time=240, color=carla.Color(255,0,0))



    # Town1
    # route 1: 107 -> 74 -> 60 -> 113
    # route 2: 0 -> 105 -> 76 -> 86 -> 104 -> 87
    # route 3: 243 -> 77 -> 31 -> 82 -> 21
    # route 4: 57 -> 111 -> 117 -> 204 -> 112 -> 230
    
    # Town2
    # route 1: 33 -> 8 -> 10 -> 12 -> 21 -> 23
    # route 2: 57 -> 70 -> 11 -> 47 -> 82 -> 55
    # route 3: 18 -> 58 -> 62 -> 66 -> 56
    # route 4: 44 -> 51 -> 72 -> 41 -> 24 -> 67
    
        # To print the route in the server map
        """ route_1_indices = [107, 74, 12, 60, 113]
        route_1 = []
        for ind in route_1_indices:
            route_1.append(spawn_points[ind].location)

        # Print the route
        world.debug.draw_string(spawn_points[route_1_indices[0]].location, "Start", life_time=1000, color=carla.Color(255,0,0))
        world.debug.draw_string(spawn_points[route_1_indices[-1]].location, "End", life_time=1000, color=carla.Color(255,0,0))
        for ind in route_1_indices:
            spawn_points[ind].location
            world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=1000, color=carla.Color(255,0,0)) """


        # get the distance between all points in route_indicies, in meters
        """ 
        total_distance = 0
        for i in range(len(route_1_indices) - 1):
            loc1 = spawn_points[route_1_indices[i]].location
            loc2 = spawn_points[route_1_indices[i+1]].location
            print(f"Idx: {route_1_indices[i]} -> {route_1_indices[i+1]} = {loc1.distance(loc2)} meters")
            
            total_distance += loc1.distance(loc2)

        print(f"Total distance: {mt.ceil(total_distance)} meters")
        """ 

        while True:
            world.tick()
            
        
        
    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")


if __name__ == '__main__':
    main()

