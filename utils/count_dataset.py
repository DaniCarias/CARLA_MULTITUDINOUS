import os

def count_imgs_data():

    validation_path = "../data/validation"
    train_path = "../data/train"

    validation_lidar = len(os.listdir(validation_path + "/lidar"))
    validation_rgb = len(os.listdir(validation_path + "/rgb"))
    validation_depth = len(os.listdir(validation_path + "/depth"))
    
    train_lidar = len(os.listdir(train_path + "/lidar"))
    train_rgb = len(os.listdir(train_path + "/rgb"))
    train_depth = len(os.listdir(train_path + "/depth"))
    
    print(f"Train Lidar: {train_lidar}")
    print(f"Train RGB: {train_rgb}")
    print(f"Train Depth: {train_depth}")
    print(f"------------")
    print(f"Validation Lidar: {validation_lidar}")
    print(f"Validation RGB: {validation_rgb}")
    print(f"Validation Depth: {validation_depth}")    
    
if __name__ == "__main__":
    count_imgs_data()
    