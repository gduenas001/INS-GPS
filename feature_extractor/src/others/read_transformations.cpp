
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <Eigen/Dense>


#include "read_matrices_pose.h"
#include "read_transformations.h"


using namespace std;


vector<Eigen::Matrix4d> read_transformations()
{
	// read poses of the car (left camera pose)
	map<int, Eigen::Matrix4d> transf_cam_to_nav_4x4;
	read_matrices_pose(transf_cam_to_nav_4x4);

	// // transformation from the velodyne to the left camera
	// Eigen::Affine3d transf_velo_to_cam_affine;
	// read_velo_to_cam(transf_velo_to_cam_affine);

	// // convert the tranform to a 4x4 matrix 
	Eigen::Matrix4d transf_velo_to_cam_4x4;
	// transf_velo_to_cam_4x4.setIdentity();
	// transf_velo_to_cam_4x4.block<3,3>(0,0)= transf_velo_to_cam_affine.rotation();
	// transf_velo_to_cam_4x4.block<3,1>(0,3)= transf_velo_to_cam_affine.translation();

	transf_velo_to_cam_4x4(0,0)= 7.533745e-03;
	transf_velo_to_cam_4x4(0,1)= -9.999714e-01;
	transf_velo_to_cam_4x4(0,2)= -6.166020e-04;
	transf_velo_to_cam_4x4(0,3)= -4.069766e-03;
	transf_velo_to_cam_4x4(1,0)= 1.480249e-02;
	transf_velo_to_cam_4x4(1,1)= 7.280733e-04;
	transf_velo_to_cam_4x4(1,2)= -9.998902e-01;
	transf_velo_to_cam_4x4(1,3)= -7.631618e-02;
	transf_velo_to_cam_4x4(2,0)= 9.998621e-01;
	transf_velo_to_cam_4x4(2,1)= 7.523790e-03;
	transf_velo_to_cam_4x4(2,2)= 1.480755e-02;
	transf_velo_to_cam_4x4(2,3)= -2.717806e-01;
	transf_velo_to_cam_4x4(3,0)= 0;
	transf_velo_to_cam_4x4(3,1)= 0;
	transf_velo_to_cam_4x4(3,2)= 0;
	transf_velo_to_cam_4x4(3,3)= 1;

	// save the total tranformation from the velo to the navigational frame in T vector
	vector<Eigen::Matrix4d> T;
	for (int i = 0; i < transf_cam_to_nav_4x4.size(); ++i)
	{
		T.push_back( transf_cam_to_nav_4x4[i]*transf_velo_to_cam_4x4 );
	}

	cout<<"Read #poses: "<< T.size()<< endl;

	return T;
}