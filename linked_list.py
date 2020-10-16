import os
import cv2
from convert_txt_pcd import get_pcd
from collections import namedtuple
import numpy as np
import open3d as o3d 

class Node:
    def __init__(self, file_lidar, file_image):
        self.img          = cv2.imread(file_image)
        self.lidar_clouds = self.get_pcd_points(file_lidar)
        self.next = None
    
    def get_pcd_points(self, my_pcd_file):
        txt_file = my_pcd_file
        my_file  = open(txt_file, "r")
        array_points = my_file.readlines()
        points       = namedtuple('Points', ['xyz', 'attr'])
        point_cloud_array = np.zeros((len(array_points), 3), dtype=np.float32)
        reflections       = np.zeros((len(array_points), 1), dtype=np.float32)

        for i, line in enumerate(array_points):
            line_array = line.split(" ")
            x = float(line_array[0])
            y = float(line_array[1])
            z = float(line_array[2].split("\n")[0])
            point_cloud_array[i, 0] = x
            point_cloud_array[i, 1] = y
            point_cloud_array[i, 2] = z
            reflections[i, 0]       = 5
        
        return points(xyz=point_cloud_array, attr=reflections)

class LinkedList:
    def __init__(self, node=None):
        self.headNode = None
        self.newNode  = node

    def insertNode(self, file_lidar, file_image):
        newNode = Node(file_lidar, file_image)
        if self.headNode == None:
            self.headNode = newNode
        else:
            current_node = self.headNode
            while(current_node.next):
                current_node  = current_node.next
            current_node.next = newNode

# first_node = Node('/home/rtml/Desktop/data/lidar/31_07_2020_07_59_46_040.txt', '/home/rtml/Desktop/data/IR_Images/CamR_31_07_2020_07_59_46_084.tiff')
# print(first_node.lidar_clouds)