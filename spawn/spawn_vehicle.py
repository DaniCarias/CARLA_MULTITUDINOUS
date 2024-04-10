import random

def spawn_vehicle(world, blueprint_library):
    # Get the blueprint for the vehicle - Tesla Model 3
    bp = blueprint_library.filter('model3')[0]
    
    # Get the spawn point
    transform = world.get_map().get_spawn_points()[15] #random.randint(0, 30) 
    
    vehicle = world.spawn_actor(bp, transform)
    
    vehicle.set_autopilot(True)
    
    return vehicle