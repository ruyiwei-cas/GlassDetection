#include "DeletePlanar.h"
#include <math.h>

bool PlaneCalculate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr objectPointCloud,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr scenePointCloud, float th)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setInputCloud(objectPointCloud);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.05);
	seg.setMaxIterations(100);
	pcl::ModelCoefficients coeff;
	pcl::PointIndices index;
	seg.segment(index, coeff);//抽取点云中距离平面小于阈值的点//也就是平面了
	;
	bool planeFlag = false;
	cout << "plane rate: " << static_cast<float>(index.indices.size()) / objectPointCloud->size() << endl;
	if (static_cast<float>(index.indices.size()) / objectPointCloud->size() > th){
		planeFlag = true;
	}

	if (!planeFlag){
		seg.setInputCloud(scenePointCloud);
		seg.segment(index, coeff);
		cout << "plane rate: " << static_cast<float>(index.indices.size()) / scenePointCloud->size() << endl;
		if (static_cast<float>(index.indices.size()) / scenePointCloud->size() > th){
			planeFlag = true;
		}
	}

	return planeFlag;
}


void DeletePlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud, float th /* = 0.1 */)
{
	
	/*std::vector<RPointCloud >V_PointCloud;
	RPointCloud point_Cloud;*/
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setInputCloud(pointCloud);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);
	seg.setMaxIterations(10);
	pcl::ModelCoefficients coeff;
	pcl::PointIndices index;
	seg.segment(index, coeff);//抽取点云中距离平面小于阈值的点//也就是平面了
	float myth = static_cast<float>(index.indices.size()) / pointCloud->size();
	float count,D,FM;
	FM = sqrt(coeff.values[0] * coeff.values[0] + coeff.values[1] * coeff.values[1] + coeff.values[2] * coeff.values[2]);
	if (myth > th)//包含平面
	{
		//把平面点云去除
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeTemp(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (int i = 0; i < pointCloud->size(); i++)
		{
			count = coeff.values[0] * pointCloud->at(i).x + coeff.values[1] * pointCloud->at(i).y + coeff.values[2] * pointCloud->at(i).z + coeff.values[3];
			
			if (count/FM > 0.01)
			{
				pcl::PointXYZRGB temppoint = pointCloud->at(i);
				temp->push_back(temppoint);
			}
			/*vector<int>::iterator it = find(index.indices.begin(), index.indices.end(), i);
			if (it == index.indices.end())
			{
				pcl::PointXYZRGB temppoint = pointCloud->at(i);
				float count;
				count = coeff.values[0] * pointCloud->at(i).x + coeff.values[1] * pointCloud->at(i).y + coeff.values[2] * pointCloud->at(i).z + coeff.values[3];
				
				if (count > 0)
				temp->push_back(temppoint);
				
				
			}*/
		
		}
		pointCloud->clear();
		pointCloud = temp;
		
	}

}

