import os
import pandas as pd

def count_imgs_data():

    test_path = "../data/test"
    train_path = "../data/train"

    test_lidar = len(os.listdir(test_path + "/lidar"))
    test_rgb = len(os.listdir(test_path + "/rgb"))
    test_depth = len(os.listdir(test_path + "/depth"))
    
    train_lidar = len(os.listdir(train_path + "/lidar"))
    train_rgb = len(os.listdir(train_path + "/rgb"))
    train_depth = len(os.listdir(train_path + "/depth"))
    
    print(f"Train Lidar: {train_lidar}")
    print(f"Train RGB: {train_rgb}")
    print(f"Train Depth: {train_depth}")
    print(f"------------")
    print(f"Test Lidar: {test_lidar}")
    print(f"Test RGB: {test_rgb}")
    print(f"Test Depth: {test_depth}")
    
    
    
if __name__ == "__main__":
    count_imgs_data()
    