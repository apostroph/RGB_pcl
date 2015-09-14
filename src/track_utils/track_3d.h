#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/projection_matrix.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

#pragma once

using namespace std;
using namespace cv;

typedef cv::Point3_<double> Point3d;

class track_3d
{
public:
	track_3d(Point3d position, double color, int size);
	
	void step();
	bool updateTracker(Point3d position, double color, int size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
	double isRecognized(Point3d position, double color, int size);
	
	int getColorHist(pcl::PointXYZRGB cloud_point);
	
	bool isAlive();
	bool isFound();
	bool isGone();
	bool isMoving();
	
	void drawTrajectory(Mat img);
	
	double getHue();
	
	double getLength();
	double getWidth();
	double getHeight();
	
	int numberOfPoints;
	
	int getNumberOfPoints();
	
	Point3d getPosition(){return position;}
	Point3d getGraspPosition(){return grasingPoint;}
	
	Point3d getVelocity(){return velocity;}
	
	pcl::PointCloud<pcl::PointXYZRGB> getPointCloud(){ return (point_cloud_tracked);}

	~track_3d(void);

private:	
	Point3d position; //Barycentric coordinate
	Point3d grasingPoint; //Barycentric coordinate
	Point3d velocity; //Average velocity of tracer object

	int approxSize; //number of pixel (or points)
	double meanColor; //Average Hue
	pcl::PointCloud<pcl::PointXYZRGB> point_cloud_tracked;
	vector<int> colors;

	int tracked_counter;
	bool tracked; //If the object is activey tracked
	bool lost; //If the object is activey tracked
	bool recognized; //If the object as been recognized
	bool acquired; // If the object is trackable
	bool moved; //If the object moved
	
	double length, width, height;
	
	double gaussian(double in, double mean, double std);
	double maximum(double x, double y, double z);
	double minimum(double x, double y, double z);
};

