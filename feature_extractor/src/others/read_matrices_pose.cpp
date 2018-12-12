
/* This function reads the poses of the car and stores them into the variable 
"matrices". Each entry in matrices is a 4x4 matrix that specifies a rotation and 
translation from the inertial frame to the current car possition. 
I: inertial frame
CAM: left camera frame (reference frame for the GPS/IMU ground truth too)
R_CAM^I: tranformation matrix from the CAM to I, i.e.
X_I = R_CAM^I * X_CAM  */

#include <iostream>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <string>
#include <Eigen/Dense>

#include "read_matrices_pose.h"

using namespace std;

int read_matrices_pose(std::map<int, Eigen::Matrix4d>& matrices)
{

	std::string filename;
	filename= "00";
	cout<< "filename: "<< filename<< endl;
	string filenameIn= "../Data/poses/00.txt";
	fstream a_file( filenameIn.c_str() , ios::in | ios::ate);
	a_file.seekg( 0, ios::beg );

    cout<< "start reading..."<< endl;
    if ( ! a_file.is_open() )
	{
	    cout << "Error opening the file" << endl;
	    return 1;
	}

	// load all the matrices in the file
	int loop= 0;
	while (1)
	{
		matrices[loop].setIdentity();
	   	double read_number;
	   	// cout<< "reading loop: "<< loop << endl;

	   	// load the matrix, first row, then columns
   		for (int row = 0; row < 3; ++row)
   		{
   			for (int column = 0; column < 4; ++column)
   			{
   				a_file >> read_number;
   				if (a_file.eof())
   				{
   					cout<< "EOF"<< endl << endl;
   					cout<< "Read: "<< loop<< " matrices."<< endl;
   					return 0;
   				}
	   			matrices[loop](row, column)= read_number;
    		}
  		}

  		// display the matrices read
		// cout << matrices[loop] << endl << endl << endl;

		loop++;
	}

	cout<< "...end reading"<< endl;

	return 0;
}