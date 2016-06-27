// test.cpp : 定义控制台应用程序的入口点。  
//  

#include "stdio.h"  
#include <opencv2/opencv.hpp>
#include "Math.h"  
#include <vector>
#include <iostream>
#include <stdarg.h> 
#include <time.h>
using namespace cv;
using namespace std;

clock_t t1, t2;
bool SortByArea(const CvRect &v1, const CvRect &v2)//注意：本函数的参数的类型一定要与vector中元素的类型一致  
{
	return v1.height*v1.width > v2.height*v2.width;//降序排列  
}
void MinRectangle(Mat Src,Mat Dst,string ImageName)
{
	static string a ;
	cout << a << endl;
	IplImage* src = cvCloneImage(&(IplImage)Src);
	IplImage *dst = cvCreateImage(cvGetSize(src), 8, 3);
	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSeq *first_contour = NULL;
	CvSeq *ru_contour = NULL; 
	
	cvFindContours(src, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	cvFindContours(src, storage, &ru_contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	cvZero(dst);
	int cnt = 0;
	vector<int >Area;

	for (; first_contour != 0; first_contour = first_contour->h_next)
	{
		cnt++;
		CvRect rect = cvBoundingRect(first_contour, 0);
		Area.push_back(rect.width*rect.height);
	}
	sort(Area.begin(), Area.end());
	first_contour = ru_contour;
	vector<CvRect> ruRect;
	int test_i = 0;
	for (; first_contour != 0; first_contour = first_contour->h_next)
	{
	
		CvRect rect = cvBoundingRect(first_contour, 0);
		
		if ((rect.width* rect.height >= Area[0]) && (rect.width* rect.height < Area[cnt - 1]) && (rect.width* rect.height >200))
			ruRect.push_back(rect);
	}
	sort(ruRect.begin(), ruRect.end(), SortByArea);
	vector<CvRect>::iterator k = ruRect.begin();
	for (int i = 1; i < ruRect.size(); i++)
	{
		if ((ruRect[0].x<ruRect[i].x) && (ruRect[0].y<ruRect[i].y) && (ruRect[0].x + ruRect[0].width>ruRect[i].x + ruRect[i].width) && (ruRect[0].y + ruRect[0].height>ruRect[i].y+ruRect[i].height))
		{
			ruRect.erase(k + i);
			i--;
		}
	}
	*dst = IplImage(Dst);
	for (int i = 0; i<(2<ruRect.size() ? 2 : ruRect.size()); i++)
	{
		cvRectangle(dst, cvPoint(ruRect[i].x*2, ruRect[i].y*2), cvPoint(ruRect[i].x*2 + ruRect[i].width*2, ruRect[i].y*2 + ruRect[i].height*2), CV_RGB(255, 0, 0), 1, 8, 0);
	}



	
	t1 = clock();
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Mat AllImage = Mat(480, 1280, CV_8UC3);
	Mat Src3C = Mat(480, 640, CV_8UC3);
	Size ImageSize;
	ImageSize.height = 480;
	ImageSize.width = 640;
	cv::resize(Src, Src, ImageSize, 0, 0, CV_INTER_NN);
	cvtColor(Src, Src3C, CV_GRAY2RGB);
	 
	Mat srcAllImage(AllImage, Rect(0, 0, 640,480));
	Src3C.copyTo(srcAllImage);

	Mat newSrcAllImage(AllImage, Rect(640, 0, 640,480));
	Mat(dst).copyTo(newSrcAllImage);

	
	cv::imshow("123", AllImage);
	cv::waitKey(1);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	t2 = clock();
	//imshow(ImageName, Mat(dst)); 
	cout << "Show Image used  time	is" << t2 - t1 << endl;

	cvReleaseMemStorage(&storage);
	/*cvWaitKey(0);*/
}