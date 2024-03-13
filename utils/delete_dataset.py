import os
import pandas as pd

def count_imgs_data():

    test_path = "../data/test"
    train_path = "../data/train"
    
    test_lidar_path = test_path + "/lidar"
    test_rgb_path = test_path + "/rgb"
    test_depth_path = test_path + "/depth"
    train_lidar_path = train_path + "/lidar"
    train_rgb_path = train_path + "/rgb"
    train_depth_path = train_path + "/depth"    
    
    for f in os.listdir(test_lidar_path):
        os.remove(os.path.join(test_lidar_path, f))
    for f in os.listdir(test_rgb_path):
        os.remove(os.path.join(test_rgb_path, f))
    for f in os.listdir(test_depth_path):
        os.remove(os.path.join(test_depth_path, f))
    for f in os.listdir(train_lidar_path):
        os.remove(os.path.join(train_lidar_path, f))
    for f in os.listdir(train_rgb_path):
        os.remove(os.path.join(train_rgb_path, f))
    for f in os.listdir(train_depth_path):
        os.remove(os.path.join(train_depth_path, f))
    
    
    
if __name__ == "__main__":
    count_imgs_data()
    print("All cleaned up!")
    