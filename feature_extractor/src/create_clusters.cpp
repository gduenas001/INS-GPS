#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <map>
#include "classes_and_structs.h"
#include "create_clusters.h"
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std;


void create_clusters (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
					  vector<pcl::PointIndices> &cluster_indices,
					  map<string, float> PARAMS)
{
	
	// create Voxels
	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	//vg.setInputCloud (cloud);
	//vg.setLeafSize (PARAMS["leafSizeXY"],PARAMS["leafSizeXY"],PARAMS["leafSizeZ"]);
	// // vg.setMinimumPointsNumberPerVoxel(2);
	//vg.filter (*cloud);

	// Eliminate far points
	pcl::PassThrough<pcl::PointXYZ> pass;
	// in X
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-PARAMS["limitsXY"], PARAMS["limitsXY"]);
	pass.filter (*cloud);
	// in Y
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-PARAMS["limitsXY"], PARAMS["limitsXY"]);
	pass.filter (*cloud);
	// in Z
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (PARAMS["limitZlow"], PARAMS["limitZhigh"]);
	pass.filter (*cloud);

	// statistical outlier removal
	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*cloud);*/


	// Remove outliers
/*	pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter (false);
	rorfilter.setInputCloud (cloud);
	rorfilter.setRadiusSearch (3*P.leafSize);
	rorfilter.setMinNeighborsInRadius (6);
	rorfilter.filter (*cloud);*/



	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (PARAMS["clusterTolerance"]);
	ec.setMinClusterSize (PARAMS["minClusterSize"]);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
 	

}