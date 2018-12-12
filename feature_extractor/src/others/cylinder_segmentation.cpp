/* This function takes the clusters and tries to find cylinders in each of them. 
Only one cylinder can be form of each of the clusters. The cylinders are stored
in the input variable "cylinders". */

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/common_headers.h>

#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <vector>


#include "classes_and_structs.h"
#include "cylinder_segmentation.h"


using namespace std;

void cylinder_segmentation (vector<Cylinder> &cylinders, 
                            vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > &clusters,
                            Eigen::Matrix4d T)
{
  // Fit cylinders to the remainding clusters
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinders; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Allocate new memory to the cloud cylinder
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointIndices::Ptr inliers_cylinder         (new pcl::PointIndices);

  // Estimate point normals
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.4); // Use neighbors in a sphere of radius 3cm
  // ne.setKSearch (50); // 50

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg_cylinders.setOptimizeCoefficients (false);
  seg_cylinders.setModelType (pcl::SACMODEL_CYLINDER);
  seg_cylinders.setMethodType (pcl::SAC_RANSAC);
  seg_cylinders.setNormalDistanceWeight (0.1); // 0.1
  seg_cylinders.setMaxIterations (10000);
  seg_cylinders.setDistanceThreshold (0.2); // 0.2
  seg_cylinders.setRadiusLimits (0.01, 0.2); // 0.1
  seg_cylinders.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
  seg_cylinders.setEpsAngle (pcl::deg2rad (10.0f));

  // Cylinder inliers and coefficients for each cluster 
  for (vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >::iterator it= clusters.begin(),
       it_end= clusters.end(); 
       it!=it_end; ++it)
  {
    // std::cout<< "Checking cluster: "<< std::distance(it, clusters.begin())
    //          << std::endl;

    // Allocate a new memory and extract there the cylinder cloud and coeffs
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> );

    // compute normals
    ne.setInputCloud (*it);
    ne.compute (*cloud_normals);

    // Segment cylinder
    seg_cylinders.setInputCloud (*it);
    seg_cylinders.setInputNormals (cloud_normals);
    seg_cylinders.segment (*inliers_cylinder, *coefficients_cylinder);

    // Extract indices
    extract.setInputCloud (*it);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);

   
    extract.filter (*cloud_cylinder);

    if (cloud_cylinder->points.empty ())  // No cylinder extracted at all
    {
      // std::cout << "Can't find the cylindrical component from cluster " 
      //           << std::distance(clusters.begin(), it) << std::endl;
    }
    /*else if (cloud_cylinder->points.size() < 18) // too small cylinder
    {
      // cout<< "Cylinder from cluster "
      //          << std::distance(clusters.begin(), it) << " is too small"<< endl;
    }*/
    else // Good cylinder -> store in cylinders
    {
      // Cylinder to push back
      Cylinder newCylinder;

      // Obtain the cross zero axis point
      Eigen::Vector4d pose(0,0,0,1);
      Eigen::Vector4d axis(0,0,0,0);
      pose[0]= coefficients_cylinder->values[0];
      pose[1]= coefficients_cylinder->values[1];
      pose[2]= coefficients_cylinder->values[2];
      axis[0]= coefficients_cylinder->values[3];
      axis[1]= coefficients_cylinder->values[4];
      axis[2]= coefficients_cylinder->values[5];

      // if the axis is points downward -> flip vector
      if (axis[2] < 0)
      {
        axis= -axis;
      }

      Eigen::Vector4d pose4x1(0,0,0,1), axis4x1(axis[0],axis[1],axis[2],0);
      Eigen::Vector2d pose2x1(0,0);

      // Project axis to the z_velo = 0 height
      pose4x1[0]= pose[0] - pose[2]*(axis[0]/axis[2]); // x velo frame
      pose4x1[1]= pose[1] - pose[2]*(axis[1]/axis[2]); // y velo frame

      // Store the measurement in local velo frame
      newCylinder.z_velo[0]= pose4x1[0]; // x velo frame
      newCylinder.z_velo[1]= pose4x1[1]; // y velo frame
      newCylinder.z_velo[2]= axis[0]; // vx velo
      newCylinder.z_velo[3]= axis[1]; // vy velo
      newCylinder.z_velo[4]= axis[2]; // vz velo

      pose4x1= T*pose4x1; // From velo to nav frame
      axis4x1= T*axis4x1;

      // Project axis to the z_nav = 0 height
      newCylinder.z_nav[0]= pose4x1[0] - pose4x1[1]*(axis4x1[0]/axis4x1[1]); // x nav frame
      newCylinder.z_nav[1]= pose4x1[2] - pose4x1[1]*(axis4x1[2]/axis4x1[1]); // z nav frame
      newCylinder.z_nav[2]= axis4x1[0];
      newCylinder.z_nav[3]= axis4x1[1];
      newCylinder.z_nav[4]= axis4x1[2];

      // pose= T*pose;
      // axis= T*axis;

      // Eigen::Vector2d pose2x1;
      // pose2x1[0]= pose[0] - pose[1]*(axis[0]/axis[1]); // x camera frame
      // pose2x1[1]= pose[2] - pose[1]*(axis[2]/axis[1]); // z camera frame

      // Store the new cylinder
      newCylinder.coefficients= coefficients_cylinder ;
      newCylinder.cloud= cloud_cylinder;
      cylinders.push_back(newCylinder);
    }

    // cout<< "Finish checking cluster: "<< std::distance(clusters.begin(), it)<< endl;
  }


  // for (int j = 0; j < cylinders.coefficients.size(); ++j)
  // {
  //   cout<< "Cylinder "<< j<< " pose: \n"<< cylinders.pose[j]<< endl;
  // }

  // // Cylinders display
  // for (int i = 0; i < cylinders.cloud.size(); ++i)
  // {
  //   std::printf("Cylinder %d \t  #points  %lu \n",
  //                                      i, cylinders.cloud[i]->points.size());
  // }
  
}