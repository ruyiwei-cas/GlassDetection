#include "CommFunc.h"

using namespace std;
using namespace cv;


bool PlaneCalculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectPointCloud,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr scenePointCloud, float th);

void DeletePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, float th /* = 0.1 */);

