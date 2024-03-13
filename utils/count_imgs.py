import os

path_rgb = '../_out/rgb/'
path_depth = '../_out/depth/'
path_lidar = '../_out/lidar/'

def count_imgs():
    print(f"Number of images in {path_rgb}: {len([name for name in os.listdir(path_rgb) if os.path.isfile(os.path.join(path_rgb, name))])}")
    print(f"Number of images in {path_depth}: {len([name for name in os.listdir(path_depth) if os.path.isfile(os.path.join(path_depth, name))])}")
    print(f"Number of images in {path_lidar}: {len([name for name in os.listdir(path_lidar) if os.path.isfile(os.path.join(path_lidar, name))])}")

def delete_imgs():
    for f in os.listdir(path_rgb):
        os.remove(os.path.join(path_rgb, f))
    for f in os.listdir(path_depth):
        os.remove(os.path.join(path_depth, f))
    for f in os.listdir(path_lidar):
        os.remove(os.path.join(path_lidar, f))

if __name__ == '__main__':
    count_imgs()
    delete_imgs()
    count_imgs()