import os
import cv2
import sys

class get_files:
    def __init__(self, data_folder):
        self.folder = data_folder
        self.img_files   = sorted(os.listdir(data_folder+"IR_Images/"))
        self.lidar_files = sorted(os.listdir(data_folder+"lidar/"))
        self.img_file_array   = []
        self.lidar_file_array = []
        self.compare_files()
    
    def compare_files(self):
        for lidar_file in self.lidar_files:
            matches = []
            for img_file in self.img_files:
                cam = img_file.split("_")[0]
                if cam == "CamR" and lidar_file[:21] == img_file[5:26]:
                    matches.append(img_file)
                # if lidar_file[:21] == img_file[:21]:
                #     matches.append(img_file)
            if len(matches) != 0:
                self.lidar_file_array.append(self.folder+"lidar/"+lidar_file)
                self.img_file_array.append(self.folder+"IR_Images/"+matches[-1])
        # print(self.lidar_file_array)

def main():
    file_path = sys.argv[1]
    get_files(file_path)

if __name__ == "__main__":
    main()