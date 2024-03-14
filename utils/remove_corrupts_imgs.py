import os
from PIL import Image
from os import listdir


def remove_corrupt_images():
    
    depth_dir = "../_out/depth"
    rgb_dir = "../_out/rgb"
    
    for file in os.listdir(depth_dir):
        file_path = os.path.join(depth_dir, file)
        try:
            img = Image.open(file_path) # open the image file
            img.verify() # verify that it is, in fact an image
        except (IOError, SyntaxError) as e:
            print('Bad file:', file_path) # print out the names of corrupt files
            os.remove(file_path)
            
    for file in os.listdir(rgb_dir):
        file_path = os.path.join(rgb_dir, file)
        try:
            img = Image.open(file_path) # open the image file
            img.verify() # verify that it is, in fact an image
        except (IOError, SyntaxError) as e:
            print('Bad file:', file_path) # print out the names of corrupt files
            os.remove(file_path)

    print("All cleaned up!")
    
    
def set_same_num_samples():
    
    imgs_depth_after = [f for f in sorted(listdir(f"../_out/depth"))]
    imgs_rgb_after = [f for f in sorted(listdir(f"../_out/rgb"))]
    imgs_lidar_after = [f for f in sorted(listdir(f"../_out/lidar"))]
    
    num_samples = min(len(imgs_depth_after), len(imgs_rgb_after), len(imgs_lidar_after))
    
    # Remove the extra images
    for i in range(num_samples, len(imgs_depth_after)):
        os.remove(f"../_out/depth/{imgs_depth_after[i]}")
    for i in range(num_samples, len(imgs_rgb_after)):
        os.remove(f"../_out/rgb/{imgs_rgb_after[i]}")
    for i in range(num_samples, len(imgs_lidar_after)):
        os.remove(f"../_out/lidar/{imgs_lidar_after[i]}")
    
    imgs_depth = [f for f in sorted(listdir(f"../_out/depth"))]
    imgs_rgb = [f for f in sorted(listdir(f"../_out/rgb"))]
    imgs_lidar = [f for f in sorted(listdir(f"../_out/lidar"))]
    
    return len(imgs_depth), len(imgs_rgb), len(imgs_lidar)
    
                
if __name__ == '__main__':
    remove_corrupt_images()
    print("All cleaned up!")
    depth, rgb, lidar = set_same_num_samples()
    print(f"Depth: {depth} | RGB: {rgb} | Lidar: {lidar}")