
#include <string>
#include <sstream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


#include "read_pcd_file.h"

using namespace std;

void read_pcd_file(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int epoch)
{
	
    // Read current file
	pcl::PCDReader reader;
	string filename2;
	stringstream sa;
	sa << setw(6) << setfill('0') << epoch;
	filename2= sa.str();
	reader.read ("../Data/pcd-files/KITTI/" + filename2 + ".pcd", *cloud);

}
