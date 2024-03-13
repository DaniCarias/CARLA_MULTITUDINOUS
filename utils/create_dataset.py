import os
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

# Loop through each sensor folder
for sensor_folder in sensor_folders:
  sensor_path = f"{out_dir}/{sensor_folder}"
  image_paths = [f"{out_dir}/{sensor_folder}/{f}" for f in listdir(sensor_path) if os.path.isfile(f"{sensor_path}/{f}")]

  # Shuffle the image paths for random selection
  random.shuffle(image_paths)

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
