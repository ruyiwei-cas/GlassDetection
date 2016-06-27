#include "CommFunc.h"
#include "DeletePlanar.h"
#include "PLY2JPEG.h"
#include "MinRectangle.h"
#include "RGBD2PCD.h"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr DownSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < cloud->size(); i++)
	{
		if ((i % 8 == 0) && (cloud->at(i).z <= 1.2))
		{
			pcl::PointXYZRGB temppoint = cloud->at(i);
			temp->push_back(temppoint);
		}
	}
	cloud->clear();
	cloud = temp;
	return cloud;
}


void cvFitPlane(const CvMat* points, float* plane){
	// Estimate geometric centroid.  
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c<ncols; c++){
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols*r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.  
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r<nrows; r++)
	for (int c = 0; c<ncols; c++)
		points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.  
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);
	// Assign plane coefficients by singular vector corresponding to smallest singular value.  
	plane[ncols] = 0;
	for (int c = 0; c<ncols; c++){
		plane[c] = V->data.fl[ncols*(ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.  
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}



int main(int argc, char* argv[])
{
	Mat image1= imread("1.jpg");
	
	image1.zeros(image1.size(), CV_8UC3);
	Mat image2 = imread("2.jpg");
	Mat image3 = imread("3.jpg");
	Mat image4 = imread("4.jpg");
	Mat image5 = imread("5.jpg");
	Mat image6 = imread("6.jpg");
	Mat image7 = imread("7.jpg");
	Mat image8 = Mat(480,640,CV_8UC1);
	Mat image9 = Mat(480,640, CV_8UC1);
	Mat image0 = Mat(480,640, CV_8UC1);
	vector<Mat > image;
	image.push_back(image8);
	image.push_back(image9);
	image.push_back(image0);
	split(image1, image);

	ofstream File;
	ifstream ImageList("ImageList.txt");
	string Address = "I:/ryw/";
	char depthImgName[100],rgbImgName[100];
	string depthName, rgbName;
	File.open("TIME.txt");
	string SaveAdd = "Image2Video/";
	clock_t T_start, T_end, T_GP, T_DP ,T_MR;
	string SaveFile;

	while (!ImageList.eof())
	{
		//pcl::PLYWriter writer;
		ImageList.getline(depthImgName, 100);
		ImageList.getline(rgbImgName, 100);
 		depthName = Address + depthImgName;
		rgbName = Address + rgbImgName;
		
		cv::Mat depthImg = loadDepthImageCompressed_RS(depthName.c_str());
		
		cout << "start:" << endl << "	" << depthName.c_str() << endl<< "	"  << rgbName.c_str() << endl << endl;
		cv::Mat rgbImg = cv::imread(rgbName.c_str());
		Mat DownRGBImg, DownDepthImg;
		Size ImageSize;
		ImageSize.height = 240;
		ImageSize.width = 320;
		cv::resize(rgbImg, DownRGBImg, ImageSize, 0, 0, CV_INTER_NN);
		cv::resize(depthImg, DownDepthImg, ImageSize, 0, 0, CV_INTER_NN);
		T_start = clock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		cloud = GeneratePointCloud(DownDepthImg, DownRGBImg, 0);

		T_GP = clock();
		cout << "Generate Point Cloud Used Time is " << T_GP - T_start << endl;

		DeletePlane(cloud, 0.1 /* = 0.8 */);
		T_DP = clock();
		cout << "Delete Plane Used Time is " << T_DP - T_GP << endl;

		if (cloud->size() <= 300) continue;
		Mat CupImage = PLY2JPEG(cloud); 
		SaveFile = SaveAdd + rgbImgName;
		MinRectangle(CupImage, rgbImg, SaveFile);
		T_MR = clock();
		cout << "Min Rectangle Used Time is " << T_MR - T_DP << endl;
		T_end = clock();
		cout << "	"<<"used time "<<T_end - T_start<< endl;
		
		
	}
	File <<"Release used time is"<< T_end - T_start <<" ms"<< endl;
	File.close();
	return 0;
}
