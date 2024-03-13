"""import os
from os import mkdir, listdir
from shutil import move
import random

data_dir = "../data"
out_dir = "../_out"

train_ratio = 0.8
test_ratio = 0.2

sensor_folders = ["rgb", "depth", "lidar"]

# Create train and test folders if they don't exist
train_path = f"{data_dir}/train"
test_path = f"{data_dir}/test"
mkdir(train_path) if not os.path.exists(train_path) else None
mkdir(test_path) if not os.path.exists(test_path) else None
for sensor_folder in sensor_folders:
  mkdir(f"{test_path}/{sensor_folder}") if not os.path.exists(f"{test_path}/{sensor_folder}") else None
  mkdir(f"{train_path}/{sensor_folder}") if not os.path.exists(f"{train_path}/{sensor_folder}") else None

# save the depth, rgb and lidar
imgs_depth = [f for f in listdir(f"{out_dir}/depth") if os.path.isfile(f"{out_dir}/depth/{f}")]
imgs_rgb = [f for f in listdir(f"{out_dir}/rgb") if os.path.isfile(f"{out_dir}/rgb/{f}")]
imgs_lidar = [f for f in listdir(f"{out_dir}/lidar") if os.path.isfile(f"{out_dir}/lidar/{f}")]


imgs_test_depth = imgs_depth[:int(len(imgs_depth) * test_ratio)]
imgs_test_rgb = imgs_rgb[:int(len(imgs_rgb) * test_ratio)]
imgs_test_lidar = imgs_lidar[:int(len(imgs_lidar) * test_ratio)]

imgs_train_depth = imgs_depth[int(len(imgs_depth) * test_ratio):]
imgs_train_rgb = imgs_rgb[int(len(imgs_rgb) * test_ratio):]
imgs_train_lidar = imgs_lidar[int(len(imgs_lidar) * test_ratio):]


for image in imgs_test_depth:
    move(out_dir + f"/depth/" + image, f"{test_path}/depth")
for image in imgs_test_rgb:
    move(out_dir + f"/rgb/" + image, f"{test_path}/rgb")
for image in imgs_test_lidar:
    move(out_dir + f"/lidar/" + image, f"{test_path}/lidar")
    
for image in imgs_train_depth:
    move(out_dir + f"/depth/" + image, f"{train_path}/depth")
for image in imgs_train_rgb:
    move(out_dir + f"/rgb/" + image, f"{train_path}/rgb")
for image in imgs_train_lidar:
    move(out_dir + f"/lidar/" + image, f"{train_path}/lidar")

print(f"Train: {len(imgs_train_depth)} depth, {len(imgs_train_rgb)} rgb, {len(imgs_train_lidar)} lidar")
print(f"Test: {len(imgs_test_depth)} depth, {len(imgs_test_rgb)} rgb, {len(imgs_test_lidar)} lidar")

"""

"""
# Loop through each sensor folder
for sensor_folder in sensor_folders:
  sensor_path = f"{out_dir}/{sensor_folder}"
  image_paths = [f"{out_dir}/{sensor_folder}/{f}" for f in listdir(sensor_path) if os.path.isfile(f"{sensor_path}/{f}")]

  # Split image paths into train and test sets based on ratio
  train_count = int(len(image_paths) * train_ratio)
  train_images = image_paths[:train_count]
  test_images = image_paths[train_count:]

  # Move images to train and test folders
  for image_path in train_images:
    move(image_path, f"{train_path}/{sensor_folder}")
  for image_path in test_images:
    move(image_path, f"{test_path}/{sensor_folder}")

  print(f"Split {sensor_folder} images: {len(train_images)} for train, {len(test_images)} for test")
"""










import os
from os import mkdir, listdir
from shutil import move

data_dir = "../data"
out_dir = "../_out"

train_ratio = 0.8
test_ratio = 0.2

sensor_folders = ["rgb", "depth", "lidar"]

# Create train and test folders if they don't exist
train_path = f"{data_dir}/train"
test_path = f"{data_dir}/test"
mkdir(train_path) if not os.path.exists(train_path) else None
mkdir(test_path) if not os.path.exists(test_path) else None

for sensor_folder in sensor_folders:
    mkdir(f"{test_path}/{sensor_folder}") if not os.path.exists(f"{test_path}/{sensor_folder}") else None
    mkdir(f"{train_path}/{sensor_folder}") if not os.path.exists(f"{train_path}/{sensor_folder}") else None

# save the depth, rgb and lidar
imgs_depth = [f for f in sorted(listdir(f"{out_dir}/depth")) if os.path.isfile(f"{out_dir}/depth/{f}")]
imgs_rgb = [f for f in sorted(listdir(f"{out_dir}/rgb")) if os.path.isfile(f"{out_dir}/rgb/{f}")]
imgs_lidar = [f for f in sorted(listdir(f"{out_dir}/lidar")) if os.path.isfile(f"{out_dir}/lidar/{f}")]

num_samples_test = int(len(imgs_depth)*test_ratio) - 1
num_samples_train = int(len(imgs_depth)*train_ratio) - 1

imgs_test_depth = imgs_depth[:num_samples_train]
imgs_test_rgb = imgs_rgb[:num_samples_train]
imgs_test_lidar = imgs_lidar[:num_samples_train]

imgs_train_depth = imgs_depth[num_samples_test:num_samples_train]  
imgs_train_rgb = imgs_rgb[num_samples_test:num_samples_train]
imgs_train_lidar = imgs_lidar[num_samples_test:num_samples_train]

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

print(f"Train: {len(imgs_train_depth)} depth, {len(imgs_train_rgb)} rgb, {len(imgs_train_lidar)} lidar")
print(f"Test: {len(imgs_test_depth)} depth, {len(imgs_test_rgb)} rgb, {len(imgs_test_lidar)} lidar")
