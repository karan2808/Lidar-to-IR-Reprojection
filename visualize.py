import os
import sys
import cv2
import yaml
import numpy as np 
import time
import math
from os.path import isfile, join
from collections import namedtuple
import open3d as o3d
from linked_list import LinkedList
from get_files import get_files

data_folder    = sys.argv[1]
roll_angle     = float(sys.argv[2])
pitch_angle    = float(sys.argv[3])
yaw_angle      = float(sys.argv[4])
x_offset       = float(sys.argv[5])
y_offset       = float(sys.argv[6])
z_offset       = float(sys.argv[7])
save_reproject = (sys.argv[8])

Points = namedtuple('Points', ['xyz', 'attr'])

def get_autoware_calib(calib_path, transform_mat = None):
    """ Get the autoware calibration matrices stored in the
    yaml file and save them in a calib dictionary. Also apply transforms
    as required."""
    calib = {}
    
    with open(calib_path, "r") as f:
        calib = yaml.load(f)

    # get the cam to velo and velo to cam transformations
    calib['cam_to_velo'] = np.reshape(calib['CameraExtrinsicMat']['data'], (calib['CameraExtrinsicMat']['rows'], calib['CameraExtrinsicMat']['cols']))
    calib['velo_to_cam'] = np.linalg.inv(calib['cam_to_velo'])
    calib['CameraMat']   = np.reshape(calib['CameraMat']['data'], (calib['CameraMat']['rows'], calib['CameraMat']['cols']))
    #print(calib['CameraMat'])
    #print(calib['cam_to_velo'])
    calib['cam_to_image']  = np.hstack([calib['CameraMat'], [[0], [0], [0]]])
    calib['velo_to_image'] = np.matmul(calib['cam_to_image'], calib['cam_to_velo'])
    calib['DistCoeff']     = np.reshape(calib['DistCoeff']['data'], (calib['DistCoeff']['rows'], calib['DistCoeff']['cols']))
    sheer = np.eye(4, 4, dtype=float)
    sheer[1, 0] = math.tan(math.radians(10))
    calib['velo_to_cam']   = np.dot(calib['velo_to_cam'], transform_mat)
    calib['velo_to_cam']   = np.dot(calib['velo_to_cam'], sheer)
    print(np.shape(calib['velo_to_cam']))
    #print(calib['velo_to_cam'])
    return calib

def velo_points_to_cam(points, calib):
    """ Convert the points in velodyne coordinates to camera coordinates"""
    velo_xyz = np.hstack([points.xyz, np.ones([points.xyz.shape[0], 1])])
    cam_xyz  = np.matmul(velo_xyz, np.transpose(calib['velo_to_cam']))[:, :3]
    return Points(xyz=cam_xyz, attr=points.attr)

def get_cam_points(velo_points, calib=None):
    """ Load the velodyne points and convert them to the camera coordinates.
    Args: frame_idx the index of the frame to read.
    Returns Points.
    """
    cam_points  = velo_points_to_cam(velo_points, calib)
    return cam_points

def cam_points_to_image(points, calib):
    """ Convert camera points to image plane.
    Args: points: a[N, 3] float32 numpy array.
    Returns: points on image plane: a[M, 2] float32 numpy array.
    a mask indicating points: a[N, 1] boolean numpy array
    """
    cam_points_xyz = np.hstack([points.xyz, np.ones([points.xyz.shape[0],1])])
    img_points_xyz = np.matmul(cam_points_xyz, np.transpose(calib['cam_to_image']))
    img_points_xyz = img_points_xyz/img_points_xyz[:,[2]]
    img_points = Points(img_points_xyz, points.attr)
    return img_points

def get_cam_points_in_image(velo_points, calib=None, image = None):
    """ Load velo points and remove the one's not observed by the camera"""
    cam_points = get_cam_points(velo_points, calib=calib)
    height = image.shape[0]
    width  = image.shape[1]
    front_cam_points_idx = cam_points.xyz[:,2]>0.1
    front_cam_points = Points(cam_points.xyz[front_cam_points_idx, :],
    cam_points.attr[front_cam_points_idx, :])
    img_points = cam_points_to_image(front_cam_points, calib)
    img_points_in_image_idx = np.logical_and.reduce(
            [img_points.xyz[:,0]>0, img_points.xyz[:,0]<width,
            img_points.xyz[:,1]>0, img_points.xyz[:,1]<height])
    cam_points_in_img = Points(
            xyz = front_cam_points.xyz[img_points_in_image_idx,:],
            attr = front_cam_points.attr[img_points_in_image_idx,:])
    return cam_points_in_img

def get_transformation_mat():
    roll_mat  = np.zeros((3, 3), dtype=float)
    pitch_mat = np.zeros((3, 3), dtype=float)
    yaw_mat   = np.zeros((3, 3), dtype=float)

    # make the roll matrix
    roll_mat[0, 0] =  1
    roll_mat[1, 1] =  math.cos(math.radians(roll_angle))
    roll_mat[2, 2] =  roll_mat[1, 1]
    roll_mat[1, 2] = -math.sin(math.radians(roll_angle))
    roll_mat[2, 1] = -roll_mat[1, 2]

    # make the pitch matrix
    pitch_mat[0, 0] =  math.cos(math.radians(pitch_angle))
    pitch_mat[2, 2] =  pitch_mat[0, 0]
    pitch_mat[1, 1] =  1
    pitch_mat[0, 2] =  math.sin(math.radians(pitch_angle))
    pitch_mat[2, 0] = -math.sin(math.radians(pitch_angle))

    # make the yaw matrix
    yaw_mat[0, 0] =  math.cos(math.radians(yaw_angle))
    yaw_mat[1, 1] =  yaw_mat[0, 0]
    yaw_mat[0, 1] =  -math.sin(math.radians(yaw_angle))
    yaw_mat[1, 0] =  -yaw_mat[0, 1]
    yaw_mat[2, 2] = 1

    mat_1 = np.dot(roll_mat, pitch_mat)
    mat_2 = np.dot(mat_1,    yaw_mat)
    translation_mat = np.array([[x_offset], [y_offset], [z_offset]])
    mat_transform = np.hstack((mat_2, translation_mat))
    mat_transform = np.vstack((mat_transform, [0., 0., 0., 1.]))
    return mat_transform

def make_linked_list():
    linked_list_obj = LinkedList()  
    get_files_obj   = get_files(data_folder)
    lidar_files     = get_files_obj.lidar_file_array
    image_files     = get_files_obj.img_file_array
    
    for ldr_file, img_file in zip(lidar_files, image_files):
        linked_list_obj.insertNode(ldr_file, img_file)
    
    return linked_list_obj

    # new_node = linked_list_obj.headNode
    # pcd      = o3d.geometry.PointCloud()

    # while(new_node.next):
    #     print(new_node.lidar_clouds.xyz)
    #     pcd.points = o3d.Vector3dVector(new_node.lidar_clouds.xyz)
    #     o3d.visualization.draw_geometries([pcd])
    #     new_node = new_node.next
def create_vis():
    vis = o3d.Visualizer()
    vis.create_window(window_name='projected point cloud', width = 640, height = 512)
    pcd        = o3d.PointCloud()
    vis.add_geometry(pcd)
    points = [[0,0,0],[1,0,0],[0,1,0],[0,0,1]]
    lines  = [[0,1],[0,2],[0,3]]
    colors = [[1,0,0],[0,1,0],[0,0,1]]
    line_set = o3d.LineSet()
    line_set.points = o3d.Vector3dVector(points)
    line_set.lines  = o3d.Vector2iVector(lines)
    line_set.colors = o3d.Vector3dVector(colors)
    vis.add_geometry(line_set)
    return vis, pcd

def main():
    new_linked_list = make_linked_list()
    calib_file    = data_folder+"calib.yaml"
    mat_transform = get_transformation_mat()
    calib         = get_autoware_calib(calib_file, mat_transform)
    vis, pcd       = create_vis()
    current_node     = new_linked_list.headNode
    j = 0
    while(current_node.next):
        cam_pts             = get_cam_points(current_node.lidar_clouds, calib=calib)
        cam_points_in_image = get_cam_points_in_image(current_node.lidar_clouds, calib=calib, image=current_node.img)
        img_points          = cam_points_to_image(cam_points_in_image, calib= calib)
        min_distance = np.min(cam_points_in_image.xyz[:,2])
        max_distance = np.max(cam_points_in_image.xyz[:,2])
        scale = 255/(max_distance-min_distance)
        # # for rgbd
        # current_node.img = cv2.cvtColor(current_node.img, cv2.COLOR_BGR2GRAY)
        point_and_image = current_node.img.copy()
        #point_and_image = 255*np.ones((512, 640, 3), np.uint8);
        depth = cam_points_in_image.xyz[:,2]
        depth_color = cv2.applyColorMap(np.uint8(scale*(depth-min_distance)), cv2.COLORMAP_JET)
        for i, img_point in enumerate(img_points.xyz):
            color = depth_color[i, 0, :].astype(np.uint8).tolist()
            cv2.circle(point_and_image, (int(img_point[0]), int(img_point[1])), 2, color, -1, 4, 0)
        width, height = int(point_and_image.shape[1]), int(point_and_image.shape[0])
        point_and_image = cv2.resize(point_and_image, (width, height))
        cv2.imwrite(data_folder+save_reproject+"image"+str(j)+".jpg", point_and_image)
        cv2.imshow('point_img', point_and_image)
        cv2.waitKey(100)
        pcd.points = o3d.Vector3dVector(cam_pts.xyz)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        current_node = current_node.next
        j = j+1
        print(j)
if __name__ == "__main__":
    main()