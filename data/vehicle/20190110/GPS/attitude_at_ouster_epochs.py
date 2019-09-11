from os import listdir
import os
import sys
import numpy as np
import csv

def main():
	dir=sys.argv[1]
	file1 = open(sys.argv[1]+"attitude_at_Ouster_frames.txt","w")
	file2 = open(sys.argv[1]+"ouster_frames_timestamps.txt","r")
	ouster_timestamps = np.loadtxt(file2)
	att_time=[]
	att_x=[]
	att_y=[]
	att_z=[]
	att_w=[]
	with open('_slash_dji_sdk_slash_attitude_edited.csv') as csvfile:
		readCSV = csv.reader(csvfile, delimiter=',')
		for row in readCSV:
			att_time.append(float(row[0]))
			att_x.append(float(row[8]))
			att_y.append(float(row[9]))
			att_z.append(float(row[10]))
			att_w.append(float(row[11]))
	att_time=np.array(att_time)
	att_time=att_time/(1e+09)
	att_x=np.array(att_x)
	att_y=np.array(att_y)
	att_z=np.array(att_z)
	att_w=np.array(att_w)

	for i in range(0,len(ouster_timestamps)):
		k=np.argmin(abs(att_time-ouster_timestamps[i,1]))
		file1.write(str(att_x[k]))
		file1.write("\t")
		file1.write(str(att_y[k]))
		file1.write("\t")
		file1.write(str(att_z[k]))
		file1.write("\t")
		file1.write(str(att_w[k]))
		file1.write("\n")

if __name__=='__main__':

	main()
