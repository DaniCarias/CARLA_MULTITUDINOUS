import random

def spawn_vehicle(world, blueprint_library, start_point=random.randint(0, 30)):
    # Get the blueprint for the vehicle - Tesla Model 3
    bp = blueprint_library.filter('model3')[0]
    
    # Get the spawn point
    transform = world.get_map().get_spawn_points()[start_point]
    
    vehicle = world.spawn_actor(bp, transform)
    
    vehicle.set_autopilot(True)
    
    return vehicle