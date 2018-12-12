#include <pcl/visualization/pcl_visualizer.h>

#include "configure_viewer.h"

// configuring point cloud viewer
pcl::visualization::PCLVisualizer configure_viewer()
{
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
	//viewer.setFullScreen(true); 
	viewer.setCameraPosition(-21.1433, -23.4669, 12.7822,0.137915, -0.429331, -1.9301,0.316165, 0.28568, 0.904669);
	viewer.setCameraClipDistances(0.0792402, 79.2402);

	return viewer;
}