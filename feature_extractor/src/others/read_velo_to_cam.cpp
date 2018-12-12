#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int read_velo_to_cam(Eigen::Affine3d &transform)
{
	string filename= 
			"/home/guille/Desktop/pcl-pr0j3ct-master/Data/calibration/calib_velo_to_cam.txt";
	fstream a_file( filename.c_str() , ios::in);
	a_file.seekg( 0, ios::beg );

	// Eigen::Affine3d transform = Eigen::Affine3d::Identity();

	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;

	cout<< "start reading..."<< endl;
    if ( ! a_file.is_open() )
      {
        cout << "Error opening the file" << endl;
        return 1;
      }

	string read_word;
	double read_number;

	while (1)
	{
		a_file >> read_word;
		if (a_file.eof())
		{
			transform.translation()= translation;
			transform.rotate(rotation);
			cout<<"rotation: \n"<< rotation<< endl;
			cout<<"translation: \n"<< translation<< endl;
			cout<<"EOF"<<endl;
			return 0;
		}

		if (!strcmp(read_word.c_str(), "R:"))
		{
			cout<< "Read rotation"<< endl;
			// Read the rotation
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					a_file >> read_number;

					rotation(i,j)= read_number;

					// cout<< read_number<< endl;
				}
			}
		}
		if (!strcmp(read_word.c_str(), "T:"))
		{
			cout<< "Read translation"<< endl;
			// Read translation
			for (int i = 0; i < 3; ++i)
			{
				a_file >> read_number;

				translation(i)= read_number;

				// cout<< read_number<< endl;
			}
		}

	}
	
	cout<< "...end reading"<< endl;

	return 0;
}