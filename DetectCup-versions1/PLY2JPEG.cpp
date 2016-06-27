#include "PLY2JPEG.h"

Mat PLY2JPEG(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	
	int x, y;
	static int i = 0;
	char name[100];
	Mat Image_Test(240, 320, CV_8UC3,Scalar(255,255,255)); 
	for (int i = 0; i < cloud->size();i++)
	{
			y = (cloud->at(i).x*310.0) / cloud->at(i).z + 159.5 + 0.5; 
			if (y <= 0)
			{
				cout << "y<=0" << endl;
				continue;
			}	
			x = (cloud->at(i).y*310.0) / cloud->at(i).z + 119.5 + 0.5;
			if (x<=0) 
			{
				cout << "x<=0" << endl;
				continue;
			}
			Image_Test.at<Vec3b>(x, y)[0] = cloud->at(i).b;
			Image_Test.at<Vec3b>(x, y)[1] = cloud->at(i).g;
			Image_Test.at<Vec3b>(x, y)[2] = cloud->at(i).r;
		
	}
	//return Image_Test;
	cv::cvtColor(Image_Test, Image_Test, CV_BGR2GRAY);
	//cv::imshow("test", Image_Test); cv::waitKey(0);
	
	cv::threshold(Image_Test, Image_Test, 250, 255, 0);	
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	Mat erosion;
	erode(Image_Test, erosion, element);
	sprintf_s(name, "Erosion/%03d.jpg", i++);
	imwrite(name, erosion);
	//imshow("Erosion", erosion);cv::waitKey(0);
	return erosion;
}

void makeBinary(cv::Mat Src, cv::Mat Binary)//将输入图片二值化
{
	int th = 250;

	cv::threshold(Src, Binary, th, 255, 0);//CV_THRESH_BINARY_INV
	Binary = Binary.clone();
	cv::imwrite("Binary.jpg", Binary);
	cv::namedWindow("Binary");
	cv::imshow("Binary", Binary);
	cv::waitKey(0);
	cv::destroyWindow("Binary");
}

void makeErosion(cv::Mat Binary, cv::Mat Erosion)//二值化图像膨胀
{
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

	erode(Binary, Erosion, element);
	Erosion = Erosion.clone();
	cv::imwrite("Erosion.jpg", Erosion);
	cv::namedWindow("Erosion");
	cv::imshow("Erosion", Erosion);
	cv::waitKey(0);
	cv::destroyWindow("Erosion");
	
}