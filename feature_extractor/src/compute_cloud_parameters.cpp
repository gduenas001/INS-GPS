#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <map>
#include <cmath>
#include "compute_cloud_parameters.h"

using namespace std;

// Compute the cluster's parameters
void compute_cloud_parameters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
							  map<string, double>& cloud_parameters)
{

    // Compute the cluster's centroid
    double meanX= 0.0, meanY= 0.0, meanZ= 0.0, minZ, maxZ;
    for (int i=0, end= cloud->points.size(); i < end; i++) // find type of object and find inbuilt function!!
    {
        if (i == 0)
        {
            minZ = double(cloud->points[i].z);
            maxZ = double(cloud->points[i].z);
        }
        else
        {
            minZ = std::min(minZ,double(cloud->points[i].z));
            maxZ = std::max(maxZ,double(cloud->points[i].z));
        }
		meanX= meanX + double(cloud->points[i].x);
		meanY= meanY + double(cloud->points[i].y);
		meanZ= meanZ + double(cloud->points[i].z);
    }
    meanX= meanX / double(cloud->points.size());
    meanY= meanY / double(cloud->points.size());
    meanZ= meanZ / double(cloud->points.size());

    // Compute the cluster's distance to optical center
    double horizontal_distance_to_optical_center=sqrt(pow(meanX,2)+pow(meanY,2));

    // Compute the cluster's variences
    double varX= 0, varY= 0, varZ=0;
    for (int i=0, end= cloud->points.size(); i < end; i++)
    {
		varX= varX + pow( double(cloud->points[i].x) - meanX, 2 );
		varY= varY + pow( double(cloud->points[i].y) - meanY, 2 );
		varZ= varZ + pow( double(cloud->points[i].z) - meanZ, 2 );
  	}
    varX= varX / double( cloud->points.size() - 1 ); cloud_parameters["sdX"]= sqrt(varX);
    varY= varY / double( cloud->points.size() - 1 ); cloud_parameters["sdY"]= sqrt(varY);
    varZ= varZ / double( cloud->points.size() - 1 ); cloud_parameters["sdZ"]= sqrt(varZ);

    // store the parameters of the cluster
    cloud_parameters["sdXYZ"]= cloud_parameters["sdX"] * cloud_parameters["sdY"] * cloud_parameters["sdZ"];
    cloud_parameters["sdXY"]= sqrt(pow(cloud_parameters["sdX"],2.0) + pow(cloud_parameters["sdY"],2.0));
    cloud_parameters["slendernessXY"]= cloud_parameters["sdZ"] / cloud_parameters["sdXY"];
    cloud_parameters["slendernessX"]= cloud_parameters["sdZ"] / cloud_parameters["sdX"];
    cloud_parameters["slendernessY"]= cloud_parameters["sdZ"] / cloud_parameters["sdY"];
    cloud_parameters["density"]= double( cloud->points.size() ) / (27*cloud_parameters["sdXYZ"]);
    cloud_parameters["meanX"]= meanX;
    cloud_parameters["meanY"]= meanY;
    cloud_parameters["minZ"]= minZ;
    cloud_parameters["maxZ"]= maxZ;
    cloud_parameters["horizontal_distance_to_optical_center"] = horizontal_distance_to_optical_center;
}