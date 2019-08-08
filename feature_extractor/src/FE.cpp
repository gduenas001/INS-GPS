// General 
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <vector>
#include <string>

// PCL specific
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <thread>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/pcl_base.h>
#include <pcl/PointIndices.h>
#include <pcl/conversions.h>
#include <locale>
#include <pcl/common/impl/io.hpp>
//#include </home/robolab/Documents/pcl/cuda/filters/include/pcl/cuda/filters/filter.h>


// User specific
#include "classes_and_structs.h"
#include "to_string.h"
#include "compute_cloud_parameters.h"
#include "create_clusters.h"
#include "configure_viewer.h"
#include "read_inputs.h"


#define VERBOSE true

#define Merge_Point_Cloud true


using namespace std;
using namespace pcl;



void read_pcd_file_callback(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int epoch)
{
	// Read current LiDAR pointclouds from two velodynes
	if (Merge_Point_Cloud)
	{
		pcl::PCLPointCloud2::Ptr cloud2_1 (new pcl::PCLPointCloud2);
		pcl::PCLPointCloud2::Ptr cloud2_2 (new pcl::PCLPointCloud2);
		pcl::PCLPointCloud2::Ptr cloud2_3 (new pcl::PCLPointCloud2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

		// Transformations used to align (and coincide) VLP1 (192.168.1.201) LiDAR pointcloud with IMU referance frame
	    Eigen::Matrix4f transform23;
	    transform23(0,0)= cos(0.0/180.0*M_PI);
	    transform23(0,1)= -1.0*sin(0.0/180.0*M_PI);
	    transform23(0,2)= 0;
	    transform23(0,3)= 0;
	    transform23(1,0)= sin(0.0/180.0*M_PI);
	    transform23(1,1)= cos(0.0/180.0*M_PI);
	    transform23(1,2)= 0;
	    transform23(1,3)= 0;
	    transform23(2,0)= 0;
	    transform23(2,1)= 0;
	    transform23(2,2)= 1.0;
	    transform23(2,3)= 0;
	    transform23(3,0)= 0;
	    transform23(3,1)= 0;
	    transform23(3,2)= 0;
	    transform23(3,3)= 1.0;
	    Eigen::Matrix4f transform22;
	    transform22(0,0)= 0;
	    transform22(0,1)= cos(10.0/180.0*M_PI);
	    transform22(0,2)= -1.0*sin(10.0/180.0*M_PI);
	    transform22(0,3)= -0.4015;
	    transform22(1,0)= -1.0;
	    transform22(1,1)= 0;
	    transform22(1,2)= 0;
	    transform22(1,3)= -0.009;
	    transform22(2,0)= 0;
	    transform22(2,1)= sin(10.0/180.0*M_PI);
	    transform22(2,2)= cos(10.0/180.0*M_PI);
	    transform22(2,3)= 0.27;
	    transform22(3,0)= 0;
	    transform22(3,1)= 0;
	    transform22(3,2)= 0;
	    transform22(3,3)= 1.0;
	    Eigen::Matrix4f transform21;
	    transform21(0,0)= 1.0;
	    transform21(0,1)= 0;
	    transform21(0,2)= 0;
	    transform21(0,3)= -0.74;
	    transform21(1,0)= 0;
	    transform21(1,1)= 1.0;
	    transform21(1,2)= 0;
	    transform21(1,3)= 0.23;
	    transform21(2,0)= 0;
	    transform21(2,1)= 0;
	    transform21(2,2)= 1.0;
	    transform21(2,3)= -1.28;
	    transform21(3,0)= 0;
	    transform21(3,1)= 0;
	    transform21(3,2)= 0;
	    transform21(3,3)= 1.0;

	    // Transformations used to align (and coincide) VLP2 (192.168.1.202) LiDAR pointcloud with IMU referance frame
	    Eigen::Matrix4f transform13;
	    transform13(0,0)= cos(1.0/180.0*M_PI);
	    transform13(0,1)= -1.0*sin(1.0/180.0*M_PI);
	    transform13(0,2)= 0;
	    transform13(0,3)= 0;
	    transform13(1,0)= sin(1.0/180.0*M_PI);
	    transform13(1,1)= cos(1.0/180.0*M_PI);
	    transform13(1,2)= 0;
	    transform13(1,3)= 0;
	    transform13(2,0)= 0;
	    transform13(2,1)= 0;
	    transform13(2,2)= 1.0;
	    transform13(2,3)= 0;
	    transform13(3,0)= 0;
	    transform13(3,1)= 0;
	    transform13(3,2)= 0;
	    transform13(3,3)= 1.0;
	    Eigen::Matrix4f transform12;
	    transform12(0,0)= 0;
	    transform12(0,1)= cos(10.0/180.0*M_PI);
	    transform12(0,2)= sin(10.0/180.0*M_PI);
	    transform12(0,3)= 0.3430;
	    transform12(1,0)= -1.0;
	    transform12(1,1)= 0;
	    transform12(1,2)= 0;
	    transform12(1,3)= 0;
	    transform12(2,0)= 0;
	    transform12(2,1)= -1.0*sin(10.0/180.0*M_PI);
	    transform12(2,2)= cos(10.0/180.0*M_PI);
	    transform12(2,3)= 0.27;
	    transform12(3,0)= 0;
	    transform12(3,1)= 0;
	    transform12(3,2)= 0;
	    transform12(3,3)= 1.0;
	    Eigen::Matrix4f transform11;
	    transform11(0,0)= 1.0;
	    transform11(0,1)= 0;
	    transform11(0,2)= 0;
	    transform11(0,3)= -0.74;
	    transform11(1,0)= 0;
	    transform11(1,1)= 1.0;
	    transform11(1,2)= 0;
	    transform11(1,3)= -0.23;
	    transform11(2,0)= 0;
	    transform11(2,1)= 0;
	    transform11(2,2)= 1.0;
	    transform11(2,3)= -0.68;
	    transform11(3,0)= 0;
	    transform11(3,1)= 0;
	    transform11(3,2)= 0;
	    transform11(3,3)= 1.0;


		pcl::PCDReader reader;
		string filename1;
		string filename2;
		stringstream sa1;
		stringstream sa2;
		sa1 << setw(6) << setfill('0') << epoch;
		sa2 << setw(6) << setfill('0') << epoch+1;
		filename1= sa1.str();
		filename2= sa2.str();

		// reading VLP1 (192.168.1.201), taking into account that even pcd files is for VLP1
		//reader.read ("../point_cloud_data/" + filename1 + ".pcd", *cloud1);
		reader.read ("/home/robolab/Robotics_Lab_experiment/src/gps_imu_lidar_data_collection/data/point_cloud_data/" + filename1 + ".pcd", *cloud1);

		// reading VLP2 (192.168.1.202), taking into account that even pcd files is for VLP2
		//reader.read ("../point_cloud_data/" + filename2 + ".pcd", *cloud2);
		reader.read ("/home/robolab/Robotics_Lab_experiment/src/gps_imu_lidar_data_collection/data/point_cloud_data/" + filename2 + ".pcd", *cloud2);

		// Transforming pointclouds of VLP1 and VLP2 to be aligned (and coincided) with IMU referance frame
		pcl::transformPointCloud (*cloud1, *cloud1, transform21);
		pcl::transformPointCloud (*cloud1, *cloud1, transform23);
		pcl::transformPointCloud (*cloud1, *cloud1, transform22);
		pcl::transformPointCloud (*cloud2, *cloud2, transform11);
		pcl::transformPointCloud (*cloud2, *cloud2, transform13);
		pcl::transformPointCloud (*cloud2, *cloud2, transform12);

		// Merging pointclouds of VLP1 and VLP2
		pcl::toPCLPointCloud2(*cloud1,*cloud2_1);
		pcl::toPCLPointCloud2(*cloud2,*cloud2_2);
		pcl::concatenatePointCloud	(*cloud2_1,*cloud2_2,*cloud2_3);
		pcl::fromPCLPointCloud2(*cloud2_3,*cloud);
	}

	// Read current LiDAR pointclouds from a single velodyne
	else
	{
		pcl::PCDReader reader;
		string filename;
		stringstream sa;
		sa << setw(6) << setfill('0') << epoch;
		filename= sa.str();
		reader.read ("../point_cloud_data/" + filename + ".pcd", *cloud);
	}

	if (VERBOSE)
		cout<< "Read next cloud: "<< epoch<< endl;
}




// *********************  MAIN  ********************* //
int main (int argc, char *argv[])
{

// Read user-defined parameters
map<string, float> PARAMS;
if (!read_inputs(PARAMS))
	cout<< "Error reading paramter files"<< endl;


// Declare variables
double meanX, meanY, meanZ, varX, varY, varZ;
string cloud_cluster_id;
map<string, double> cloud_parameters;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

int initial_frame= static_cast<int>(PARAMS["initial_frame"]);
int num_frames= static_cast<int>(PARAMS["num_frames"]);

// used to make a proper numbering for Feature Extractor's output; in the case of having two LiDARs
int count =initial_frame; // count refers to timestep

// Initializing PCL visualizer
pcl::visualization::PCLVisualizer viewer= configure_viewer();

// create vector of threads
vector<thread> threadVector;

// Read first cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr next_cloud (new pcl::PointCloud<pcl::PointXYZ>);
threadVector.push_back(thread( read_pcd_file_callback, next_cloud, initial_frame ));	


// epoch refers to pointcloud file number (not timestep)
// for (int epoch= initial_frame; epoch < num_frames; ++epoch) // used in case of having one LiDAR
for (int epoch= initial_frame; epoch < num_frames; epoch=epoch+2) // used in case of having two LiDARs
{

	//------------------ READING PCD FILE ------------------//

	// join previous thread
	threadVector.at( threadVector.size()-1 ).join();
	cloud= next_cloud;

	// Start next thread
	next_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

	// threadVector.push_back(thread (read_pcd_file_callback, next_cloud, epoch+1)); // used in case of having one LiDAR
	threadVector.push_back(thread (read_pcd_file_callback, next_cloud, epoch+2)); // used in case of having two LiDARs

	// Reduce size of vector of threads
	if (threadVector.size() > 2)
		threadVector.erase( threadVector.begin() + 1 );
	//------------------------------------------------------//
	


	// Visualize point cloud - White cloud
	viewer.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
					white_color (cloud, 255, 255, 255);
	viewer.addPointCloud <pcl::PointXYZ> (cloud, white_color, "cloud_original");




	// Extraction of clusters
	vector<pcl::PointIndices> clusters_indices;
	create_clusters(cloud, clusters_indices, PARAMS);
	 

	vector <pcl::PointCloud<pcl::PointXYZ> ::Ptr> clusters; // Vector of pointclouds pointers
	

	// Extract cluster pointcloud from indeces
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setNegative (false);

	// Opening a file to store extracted features at epoch(count)
	ofstream myfile;
	myfile.open ("../Feature_Extractor_output/Epoch" + std::to_string(count) + ".txt");

	int numClusters= 0;

	for (int cluster_index = 0; cluster_index < clusters_indices.size(); ++cluster_index)
	{
		// Extract the inliers of the cluster
	    pcl::PointIndices::Ptr cluster_indeces_pointer
	    					(new pcl::PointIndices ( clusters_indices[cluster_index] ));
    	extract.setIndices (cluster_indeces_pointer);

    	// Extract cluster pointcloud from indeces
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster 
    						(new pcl::PointCloud<pcl::PointXYZ>);
    	extract.filter (*cloud_cluster);

    	// compute parameters of the cluster's pointcloud
	    compute_cloud_parameters(cloud_cluster, cloud_parameters);

	    //Normal mode (extract cylindrical features)
	    if (!PARAMS["Basic"]){

	    // Check if the cluster is a cylindrical feature
	    if (
	    	 // sdXY: standard deviation of the cluster in the radial direction
	    	 cloud_parameters["sdXY"] < PARAMS["sdXYThreshold"]				   &&
	    	 // slendernessXY: ratio between sdZ and sdXY
	    	 cloud_parameters["slendernessXY"] > PARAMS["slindernessThreshold"] &&
	    	 // reject clusters that are very close to the LiDAR (optical center)
	    	 cloud_parameters["horizontal_distance_to_optical_center"] > PARAMS["min_horizontal_distance"] &&
	    	 // reject clusters that are shorter than a specific threshold
	    	 cloud_parameters["maxZ"] > PARAMS["cluster_maxZ"])
	    {
		    	// Add cluster
			    ++numClusters; 
			    clusters.push_back(cloud_cluster);

			// write the cluster centroid in the output file
			    myfile << std::to_string(cloud_parameters["meanX"]) << "\t" << std::to_string(cloud_parameters["meanY"]) << "\n";

			    if (VERBOSE)
				{
				    cout << "Cluster " << numClusters << " ----> " 
				    					<< clusters[numClusters-1]->points.size() << " points." << endl;
				    cout << "density = " << cloud_parameters["density"] << endl;
				    cout << "cluster_maxZ = " << cloud_parameters["maxZ"] << endl;
				    cout << "SD in X = " << cloud_parameters["sdX"] << endl;
				    cout << "SD in Y = " << cloud_parameters["sdY"] << endl;
				    cout << "SD in Z = " << cloud_parameters["sdZ"] << endl;
				    cout << "slendeness in X = " << cloud_parameters["slendernessX"] << endl;
				    cout << "slendeness in Y = " << cloud_parameters["slendernessY"] << endl << endl;
		    	}
		}
							}

		//Basic mode (extract all clusters; cylindrical and non-cylindrical)
		else{
				// Add cluster
				++numClusters;
			    clusters.push_back(cloud_cluster);

			    if (VERBOSE)
				{
				    cout << "Cluster " << numClusters << " ----> " 
				    					<< clusters[numClusters-1]->points.size() << " points." << endl;
				    cout << "density = " << cloud_parameters["density"] << endl;
				    cout << "cluster_maxZ = " << cloud_parameters["maxZ"] << endl;
				    cout << "SD in X = " << cloud_parameters["sdX"] << endl;
				    cout << "SD in Y = " << cloud_parameters["sdY"] << endl;
				    cout << "SD in Z = " << cloud_parameters["sdZ"] << endl;
				    cout << "slendeness in X = " << cloud_parameters["slendernessX"] << endl;
				    cout << "slendeness in Y = " << cloud_parameters["slendernessY"] << endl << endl;
				}
			}

	} // End of loop over clusters

	// close the file that stores the cluster's centroid at the current timestep
	myfile.close();

	// Jumping to the next timestep
	count = count + 1;

	// visualize the clusters using red colour
	int counter= 0;
	for (vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator it= clusters.begin(),
       it_end= clusters.end(); 
       it!=it_end; ++it, ++counter)
  	{
  		cloud_cluster_id= "cloud_cluster_" + patch::to_string( counter );
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (*it, 255, 0, 0);
		viewer.addPointCloud <pcl::PointXYZ> (*it, red_color, cloud_cluster_id);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_cluster_id);
	}

	cout<< "-----------------------------------"<< endl;
	cout << "# epoch = " << epoch << endl;
	cout << "# clusters = " << numClusters << endl;
	cout<< "-----------------------------------"<< endl;


	// Stopping Viewer on specific timestep (count)
	if (PARAMS["Debug"]){
		while (1){
			viewer.spinOnce ();
			}
		}
		viewer.spinOnce ();


}

} // End of Main
