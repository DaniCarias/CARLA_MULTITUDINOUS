import cv2
import os

for img in os.listdir("./_out/ground_truth/depth1/"):
    print(f"Processing image {img}")
    img1 = cv2.imread(f"./_out/ground_truth/depth1/{img}")
    img2 = cv2.imread(f"./_out/ground_truth/depth2/{img}")
    img3 = cv2.imread(f"./_out/ground_truth/depth3/{img}")
    
    img_concat = cv2.hconcat([img1, img2, img3])
    # create a png file with the concatenated images
    
    flag = cv2.imwrite(f"./_out/ground_truth/depth_concat/{img}", img_concat)
    print(f"Image {img} concatenated successfully! -> {flag}")