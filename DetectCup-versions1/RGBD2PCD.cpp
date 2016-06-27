#include "RGBD2PCD.h"

pcl::PointXYZRGB Point2DTo3D(const cv::KeyPoint& point2D, uint16_t idensity,
	float focalLength = 310.0, float centerX = 159.5, float centerY = 119.5, float scalingFactor = 1000.0)
{
	pcl::PointXYZRGB pt;
	pt.z = idensity/ scalingFactor;
	pt.x = (point2D.pt.x - centerX)*pt.z / focalLength;
	pt.y = (point2D.pt.y - centerY)*pt.z / focalLength;
	
	return pt;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr GeneratePointCloud(const cv::Mat& depthImg,
	const cv::Mat& rgbImg, float leafSize = 0.0)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			unsigned short identity = depthImg.at<unsigned short>(i, j);
			if (identity <= 0.500) continue;
			cv::KeyPoint kpt;
			kpt.pt.x = j; kpt.pt.y = i;
			pcl::PointXYZRGB tpt = Point2DTo3D(kpt, identity);
			if (tpt.z > 1.200) continue;
			ptCloud->push_back(tpt);
			ptCloud->back().b = rgbImg.at<cv::Vec3b>(i, j)[0];
			ptCloud->back().g = rgbImg.at<cv::Vec3b>(i, j)[1];
			ptCloud->back().r = rgbImg.at<cv::Vec3b>(i, j)[2];
		}
	}

	if (abs(leafSize) > 0.001)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptSampleCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setLeafSize(leafSize, leafSize, leafSize);
		sor.setInputCloud(ptCloud);
		sor.filter(*ptSampleCloud);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptFilterCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
		filter.setInputCloud(ptSampleCloud);
		filter.setMeanK(50);
		filter.setStddevMulThresh(1.0);
		filter.filter(*ptFilterCloud);

		return ptFilterCloud;
	}


	return ptCloud;
}


cv::Mat_<ushort> loadDepthImageCompressed_RS(const char* fname)
{
	FILE* pFile = fopen(fname, "rb");
	ushort *depthData = NULL;
	if (NULL == pFile)
	{
		std::cerr << "Could not open file " << fname << std::endl;
		return cv::Mat_<ushort>();
	}

	int imWidth(640), imHeight(480);

	if (depthData)
	{
		delete[] depthData;
		depthData = NULL;
	}
	depthData = new ushort[imHeight * imWidth];

	bool success(false);

	fread(depthData, sizeof(ushort), imWidth * imHeight, pFile);

	fclose(pFile);

	cv::Mat_<ushort> depthImg;
	depthImg.rows = imHeight;
	depthImg.cols = imWidth;
	depthImg.create(imHeight, imWidth);


	memcpy(depthImg.data, depthData, sizeof(ushort)* imHeight * imWidth);
	delete[] depthData;
	return depthImg;
}

int PCDtoPLYconvertor(char* input_filename, char* output_filename)
{
	pcl::PCLPointCloud2 cloud;
	if (pcl::io::loadPCDFile(input_filename, cloud) < 0)
	{
		cout << "Error: cannot load the PCD file!!!" << endl;
		return -1;
	}
	pcl::PLYWriter writer;
	writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
	return 0;

}

