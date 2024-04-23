import os

path_degree_0_rgb = '../_out/ground_truth/degree_0/rgb'
path_degree_0_depth = '../_out/ground_truth/degree_0/depth'
path_degree_120_rgb = '../_out/ground_truth/degree_120/rgb'
path_degree_120_depth = '../_out/ground_truth/degree_120/depth'
path_degree_240_rgb = '../_out/ground_truth/degree_240/rgb'
path_degree_240_depth = '../_out/ground_truth/degree_240/depth'


def delete_imgs():
    for f in os.listdir(path_degree_0_rgb):
        os.remove(os.path.join(path_degree_0_rgb, f))
    for f in os.listdir(path_degree_0_depth):
        os.remove(os.path.join(path_degree_0_depth, f))
    for f in os.listdir(path_degree_120_rgb):
        os.remove(os.path.join(path_degree_120_rgb, f))
    for f in os.listdir(path_degree_120_depth):
        os.remove(os.path.join(path_degree_120_depth, f))
    for f in os.listdir(path_degree_240_rgb):
        os.remove(os.path.join(path_degree_240_rgb, f))
    for f in os.listdir(path_degree_240_depth):
        os.remove(os.path.join(path_degree_240_depth, f))

if __name__ == '__main__':
    delete_imgs()
    