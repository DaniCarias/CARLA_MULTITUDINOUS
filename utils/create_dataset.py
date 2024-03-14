import os
from os import mkdir, listdir
from shutil import move
import math

data_dir = "../data"
out_dir = "../_out"

train_ratio = 0.8
test_ratio = 0.2

sensor_folders = ["rgb", "depth", "lidar"]

train_path = f"{data_dir}/train"
test_path = f"{data_dir}/test"

# Create train and test folders if they don't exist
mkdir(train_path) if not os.path.exists(train_path) else None
mkdir(test_path) if not os.path.exists(test_path) else None
for sensor_folder in sensor_folders:
    mkdir(f"{test_path}/{sensor_folder}") if not os.path.exists(f"{test_path}/{sensor_folder}") else None
    mkdir(f"{train_path}/{sensor_folder}") if not os.path.exists(f"{train_path}/{sensor_folder}") else None

# save the depth, rgb and lidar
imgs_depth = [f for f in sorted(listdir(f"{out_dir}/depth"))]
imgs_rgb = [f for f in sorted(listdir(f"{out_dir}/rgb"))]
imgs_lidar = [f for f in sorted(listdir(f"{out_dir}/lidar"))]


num_samples = min(len(imgs_depth), len(imgs_rgb), len(imgs_lidar))
num_samples_test = math.floor(int(num_samples*test_ratio))
num_samples_train = math.floor(int(num_samples*train_ratio))
print(f"Total num samples of each sensor: {num_samples}")
print(f"num_samples_test: {num_samples_test}")
print(f"num_samples_train: {num_samples_train}")


imgs_test_depth = imgs_depth[:num_samples_test]
imgs_test_rgb = imgs_rgb[:num_samples_test]
imgs_test_lidar = imgs_lidar[:num_samples_test]
print(f"\n-----------TEST-----------")
print(f"imgs_test_depth: {len(imgs_test_depth)}")
print(f"imgs_test_rgb: {len(imgs_test_rgb)}")
print(f"imgs_test_lidar: {len(imgs_test_lidar)}")


imgs_train_depth = imgs_depth[num_samples_test:num_samples_train]
imgs_train_rgb = imgs_rgb[num_samples_test:num_samples_train]
imgs_train_lidar = imgs_lidar[num_samples_test:num_samples_train]
print(f"\n-----------TRAIN-----------")
print(f"imgs_train_depth: {len(imgs_train_depth)}")
print(f"imgs_train_rgb: {len(imgs_train_rgb)}")
print(f"imgs_train_lidar: {len(imgs_train_lidar)}")

for image in imgs_test_depth:
    move(f"{out_dir}/depth/{image}", f"{test_path}/depth")
for image in imgs_test_rgb:
    move(f"{out_dir}/rgb/{image}", f"{test_path}/rgb")
for image in imgs_test_lidar:
    move(f"{out_dir}/lidar/{image}", f"{test_path}/lidar")

for image in imgs_train_depth:
    move(f"{out_dir}/depth/{image}", f"{train_path}/depth")
for image in imgs_train_rgb:
    move(f"{out_dir}/rgb/{image}", f"{train_path}/rgb")
for image in imgs_train_lidar:
    move(f"{out_dir}/lidar/{image}", f"{train_path}/lidar")

print(f"\nTrain: {len(imgs_train_depth)} depth, {len(imgs_train_rgb)} rgb, {len(imgs_train_lidar)} lidar")
print(f"Test: {len(imgs_test_depth)} depth, {len(imgs_test_rgb)} rgb, {len(imgs_test_lidar)} lidar")
