from os import listdir
import os
import sys

def main():
	file = open(sys.argv[2]+"ouster_frames_timestamps.txt","w")
	file.write("frame number")
	file.write(",")
	file.write("timestamp")
	file.write("\n")
	dir=sys.argv[1]
	# dir="/home/robotics/Robotics_Lab_experiment/src/gps_imu_lidar_data_collection/data/point_cloud_data/"
	list=sorted(listdir(dir))
	number_of_pcd_files=len(list)
	list_float=[]
	for i in range(0,number_of_pcd_files):
		list_float.insert(len(list_float),float(list[i][0:len(list[i])-4]))
		file.write(str(i))
		file.write(",")
		file.write(str(list_float[i]))
		file.write("\n")
		os.rename(dir+list[i],dir+str(i)+".txt")

if __name__=='__main__':

	main()
