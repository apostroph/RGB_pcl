#include "track_3d.h"


track_3d::track_3d(Point3d in_position, double color, int size)
{
	this->position = in_position; //Barycentric coordinate
	grasingPoint = in_position; //Grasping point initialised as barycenter
	this->velocity = Point3d(0, 0, 0); //Average velocity of tracer object

	this->approxSize = size; //number of pixel (or points)
	this->meanColor = color; //Average Hue

	this->tracked = false;
	this->recognized = true;
	this->acquired = true;
	this->lost = false;
	this->moved = false;
	
	tracked_counter = 0;
}

double track_3d::isRecognized(Point3d position, double color, int size){
	recognized = false;
	
	//compare x axis
	double xScore = gaussian(abs(this->position.x - position.x), 0, 0.05+abs(this->velocity.x))/gaussian(0, 0, 0.05+abs(this->velocity.x));
	
	//compare y axis
	double yScore = gaussian(abs(this->position.y - position.y), 0, 0.05+abs(this->velocity.y))/gaussian(0, 0, 0.05+abs(this->velocity.y));
	
	//compare z axis
	double zScore = gaussian(abs(this->position.z - position.z), 0, 0.05+abs(this->velocity.z))/gaussian(0, 0,  0.05+abs(this->velocity.z));
	
	//compare color
	double cScore = gaussian(abs(this->meanColor - color), 0, 15)/gaussian(0, 0, 15);
	
	//compare size
	double sScore = gaussian(abs(this->approxSize - size), 0, 30)/gaussian(0, 0, 30);
	
	//estimate score
	double score = sqrt( xScore * yScore * zScore * cScore * sScore );
	
// 	cout<<score<<endl;
	return score;
}

void track_3d::step(){
	
	if(recognized){
		tracked_counter++;
	}else{
		tracked_counter--;
	}
  
	if(tracked_counter >= 2){
		tracked = true;
		tracked_counter = 2;
	}else if(tracked_counter < -4){
		tracked = false;
		lost = true;
	}
	
	acquired = false;
	recognized = false;
}

bool track_3d::updateTracker(Point3d position, double color, int size, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud){
	
	isRecognized(position, color, size);
  
	this->velocity = this->position - position;
	this->position = position;
	this->meanColor = color;
	this->approxSize = size;
	
	double vel = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2) + pow(velocity.z, 2));
	
	if(vel > 0.05){
		moved = true;
	}else{
		moved = false;
	}
	
	double limit_x[2] = {DBL_MIN, DBL_MAX};
	double limit_y[2] = {DBL_MIN, DBL_MAX};
	double limit_z[2] = {DBL_MIN, DBL_MAX};
	
	for(auto cloud_point: pointCloud->points){
		if(cloud_point.x > limit_x[0]){
			limit_x[0] = cloud_point.x;
		}else if(cloud_point.x < limit_x[1]){
			limit_x[1] = cloud_point.x;
		}
		if(cloud_point.y > limit_y[0]){
			limit_y[0] = cloud_point.y;
		}else if(cloud_point.y < limit_y[1]){
			limit_y[1] = cloud_point.y;
		}
		if(cloud_point.z > limit_z[0]){
			limit_z[0] = cloud_point.z;
		}else if(cloud_point.z < limit_z[1]){
			limit_z[1] = cloud_point.z;
		}
	}
	length = limit_x[0]-limit_x[1];
	width  = limit_y[0]-limit_y[1];
	height = limit_z[0]-limit_z[1];
	
	grasingPoint.x = limit_x[1];
	grasingPoint.y = position.y;
	grasingPoint.z = limit_z[0];
	
	numberOfPoints = pointCloud->points.size();
	
	point_cloud_tracked = *pointCloud;
	
	recognized = true;
	acquired = true;
	
	return acquired;
}

bool track_3d::isAlive(){
     return !lost;
}

bool track_3d::isGone(){
	if(tracked_counter < 0)
		return true;
	return false;
}


bool track_3d::isFound(){
	return tracked;
}
 
bool track_3d::isMoving(){
	return moved;
}
 
double track_3d::getHue(){
      return meanColor;
  
}

double track_3d::getLength(){
     return length;
}

double track_3d::getWidth(){
     return width;
}

double track_3d::getHeight(){
     return height;
}

int track_3d::getNumberOfPoints(){
    return numberOfPoints;
}



double track_3d::gaussian(double in, double mean, double std)
{
      return (1./(std*sqrt(2*M_PI)))*exp(-0.5*(pow(in-mean, 2)/(pow(std,2))));
}

track_3d::~track_3d(void)
{
}
