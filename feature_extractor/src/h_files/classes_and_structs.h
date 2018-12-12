

// Cylinder 
struct Cylinder
{
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	Eigen::Matrix<double, 5, 1> z_velo, z_nav;
};


// Frame
struct Frame
{
	int numFeatures;
	int numDetected;
	int numExpected;
	double repRate;

	std::vector<int> association;

	std::vector< Eigen::Matrix<double, 5, 1> > z_velo, z_nav;

	Frame();
};

// Landmark
struct Landmark
{
	Eigen::Matrix<double, 5, 1> pose;
	int rep;
	int expected;
	int expected_now;

	Landmark();
};

