#include <stdint.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "CommFunc.h"

using namespace std;
using namespace cv;

pcl::PointXYZRGB Point2DTo3D(const cv::KeyPoint& point2D, uint16_t idensity,
	float focalLength, float centerX, float centerY, float scalingFactor);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloud(const cv::Mat& depthImg,
	const cv::Mat& rgbImg, float leafSize);

cv::Mat_<ushort> loadDepthImageCompressed_RS(const char* fname);

int PCDtoPLYconvertor(char* input_filename, char* output_filename);

