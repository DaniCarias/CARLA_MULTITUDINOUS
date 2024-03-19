import os

def count_imgs_data():

    validation_path = "../data/validation"
    train_path = "../data/train"
    
    validation_lidar_path = validation_path + "/lidar"
    validation_rgb_path = validation_path + "/rgb"
    validation_depth_path = validation_path + "/depth"
    train_lidar_path = train_path + "/lidar"
    train_rgb_path = train_path + "/rgb"
    train_depth_path = train_path + "/depth"    
    
    for f in os.listdir(validation_lidar_path):
        os.remove(os.path.join(validation_lidar_path, f))
    for f in os.listdir(validation_rgb_path):
        os.remove(os.path.join(validation_rgb_path, f))
    for f in os.listdir(validation_depth_path):
        os.remove(os.path.join(validation_depth_path, f))
    for f in os.listdir(train_lidar_path):
        os.remove(os.path.join(train_lidar_path, f))
    for f in os.listdir(train_rgb_path):
        os.remove(os.path.join(train_rgb_path, f))
    for f in os.listdir(train_depth_path):
        os.remove(os.path.join(train_depth_path, f))
    
    
    
if __name__ == "__main__":
    count_imgs_data()
    print("All cleaned up!")
    