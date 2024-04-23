import open3d
import os

PCL_PATH = '../_out/lidar_teste/'

def get_pointcloud(file):
    
    with open(PCL_PATH + file, 'r') as f:
        pcl = f.readlines()

    points = []

    for point in pcl[13:]:
        data = point.strip().split()
        x = float(data[0])
        y = float(data[1])
        z = float(data[2])
        class_tag = int(data[-1])
        points.append([x, y, z, class_tag])
        
    return points
    
    
    
if __name__ == '__main__':
    
    pcls_points = []
    
    for file in os.listdir(PCL_PATH):
        print(file)
        points = get_pointcloud(file)
        pcls_points.append(points)
    
    print(pcls_points[:2])