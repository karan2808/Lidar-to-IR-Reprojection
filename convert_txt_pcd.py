import sys
import open3d as o3d 
import numpy as np
from collections import namedtuple

class get_pcd:

    def __init__(self, file_path):
        self.my_pcd_file = file_path
        self.points      = namedtuple('Points', ['xyz', 'attr'])
        self.get_pcd_points()

    def get_pcd_points(self):
        txt_file = self.my_pcd_file
        my_file  = open(txt_file, "r")
        array_points = my_file.readlines()

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
        
        self.points(xyz=point_cloud_array, attr=reflections)
        # print(point_cloud_array)
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(point_cloud_array)
        # o3d.visualization.draw_geometries([pcd])