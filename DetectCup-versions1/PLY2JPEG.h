#include "CommFunc.h"
#include <fstream>
#include <stdio.h>

using namespace std;
using namespace cv;

Mat PLY2JPEG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void makeBinary(cv::Mat Src, cv::Mat Binary);
void makeErosion(cv::Mat Binary, cv::Mat Erosion);