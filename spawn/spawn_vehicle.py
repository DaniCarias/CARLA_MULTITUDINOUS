import random

def spawn_vehicle(world, blueprint_library):
    # Get the blueprint for the vehicle - Tesla Model 3
    bp = blueprint_library.filter('model3')[0]
    # Get the spawn point
    transform = world.get_map().get_spawn_points()[random.randint(0, 30)]
    # Spawn the vehicle
    vehicle = world.spawn_actor(bp, transform)
    # Set the autopilot
    vehicle.set_autopilot(True)
    
    return vehicle