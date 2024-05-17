from setup import setup_world
from spawn import spawn_vehicle
import carla
import queue

def spawn_cameras(camera, world, blueprint_library, vehicle, img_width, img_height, camera_transform):
    camera_bp = blueprint_library.find(camera)
    
    camera_bp.set_attribute('image_size_x', f"{img_width}")
    camera_bp.set_attribute('image_size_y', f"{img_height}")
    
    camera_bp.set_attribute('fov', f"{90}")
    camera_bp.set_attribute('lens_k', f"{0}")               # Remove distortion
    camera_bp.set_attribute('lens_kcube', f"{0}")           # Remove distortion
    camera_bp.set_attribute('lens_circle_falloff', f"{0}")

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

def main():
    actor_list = []
    IMG_WIDTH = 800 #1280
    IMG_HEIGHT = 600 #720

    try:
        world, blueprint_library, traffic_manager = setup_world.setup_carla()
        settings = world.get_settings()
        
        settings.fixed_delta_seconds = 1
        settings.no_rendering_mode = True # No rendering mode
        settings.synchronous_mode = True # Enables synchronous mode
        world.apply_settings(settings)

            
    # Vehicle
        vehicle = spawn_vehicle.spawn_vehicle(world, blueprint_library)
        #vehicle.set_autopilot(False)
        actor_list.append(vehicle)
        
        # Ignore all the red ligths
        traffic_manager.ignore_lights_percentage(vehicle, 100)
        
        
    # Depth & RGB - 1
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5))
        camera_depth1 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb1 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)

    # Depth & RGB - 2
        rotation60 = carla.Rotation(yaw=60.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation60)
        camera_depth2 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb2 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)

    # Depth & RGB - 3
        rotation120 = carla.Rotation(yaw=120.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation120)
        camera_depth3 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb3 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        
    # Depth & RGB - 4
        rotation180 = carla.Rotation(yaw=180.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation180)
        camera_depth4 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb4 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        
    # Depth & RGB - 5
        rotation240 = carla.Rotation(yaw=240.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation240)
        camera_depth5 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb5 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        
    # Depth & RGB - 6
        rotation300 = carla.Rotation(yaw=300.0)
        camera_transform = carla.Transform(carla.Location(x=0.9, z=2.5), rotation300)
        camera_depth6 = spawn_cameras('sensor.camera.depth', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)
        camera_rgb6 = spawn_cameras('sensor.camera.rgb', world, blueprint_library, vehicle, IMG_WIDTH, IMG_HEIGHT, camera_transform)


        actor_list.append([camera_depth1, camera_rgb1, camera_depth2, camera_rgb2, camera_depth3, camera_rgb3, camera_depth4, camera_rgb4, camera_depth5, camera_rgb5, camera_depth6, camera_rgb6])

        image_queue_depth1 = queue.Queue()
        image_queue_depth2 = queue.Queue()
        image_queue_depth3 = queue.Queue()
        image_queue_depth4 = queue.Queue()
        image_queue_depth5 = queue.Queue()
        image_queue_depth6 = queue.Queue()
        image_queue_rgb1 = queue.Queue()
        image_queue_rgb2 = queue.Queue()
        image_queue_rgb3 = queue.Queue()
        image_queue_rgb4 = queue.Queue()
        image_queue_rgb5 = queue.Queue()
        image_queue_rgb6 = queue.Queue()
        

        camera_depth1.listen(image_queue_depth1.put)
        camera_depth2.listen(image_queue_depth2.put)
        camera_depth3.listen(image_queue_depth3.put)
        camera_depth4.listen(image_queue_depth4.put)
        camera_depth5.listen(image_queue_depth5.put)
        camera_depth6.listen(image_queue_depth6.put)
        camera_rgb1.listen(image_queue_rgb1.put)
        camera_rgb2.listen(image_queue_rgb2.put)
        camera_rgb3.listen(image_queue_rgb3.put)
        camera_rgb4.listen(image_queue_rgb4.put)
        camera_rgb5.listen(image_queue_rgb5.put)
        camera_rgb6.listen(image_queue_rgb6.put)

        while True:
            world.tick()
            cc = carla.ColorConverter.LogarithmicDepth
            
            image = image_queue_depth1.get()
            print(f"Frame: {image.frame} | IMG: {image} | RGBA: {image.raw_data}")
            image.save_to_disk('_out/ground_truth/degree_0/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb1.get()
            image.save_to_disk('_out/ground_truth/degree_0/rgb/%06d' % image.frame + '.png')
            
            image = image_queue_depth2.get()
            image.save_to_disk('_out/ground_truth/degree_60/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb2.get()
            image.save_to_disk('_out/ground_truth/degree_60/rgb/%06d' % image.frame + '.png')

            image = image_queue_depth3.get()
            image.save_to_disk('_out/ground_truth/degree_120/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb3.get()
            image.save_to_disk('_out/ground_truth/degree_120/rgb/%06d' % image.frame + '.png')
            
            image = image_queue_depth4.get()
            image.save_to_disk('_out/ground_truth/degree_180/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb4.get()
            image.save_to_disk('_out/ground_truth/degree_180/rgb/%06d' % image.frame + '.png')
            
            image = image_queue_depth5.get()
            image.save_to_disk('_out/ground_truth/degree_240/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb5.get()
            image.save_to_disk('_out/ground_truth/degree_240/rgb/%06d' % image.frame + '.png')
            
            image = image_queue_depth6.get()
            image.save_to_disk('_out/ground_truth/degree_300/depth/%06d' % image.frame + '.png', cc)
            image = image_queue_rgb6.get()
            image.save_to_disk('_out/ground_truth/degree_300/rgb/%06d' % image.frame + '.png')
        
    finally:
        for actor in actor_list:
            actor.destroy()
        print(f"All cleaned up!")


if __name__ == '__main__':
    main()