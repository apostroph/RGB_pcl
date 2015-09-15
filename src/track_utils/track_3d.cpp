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
	
	double limit_x[2] = {pointCloud->points[0].x, pointCloud->points[0].x};
	double limit_y[2] = {pointCloud->points[0].y, pointCloud->points[0].y};
	double limit_z[2] = {pointCloud->points[0].z, pointCloud->points[0].z};
	
	
	colors.clear();
	for(int c = 0; c < 5; c++){
		colors.push_back(0);
	}
	
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
		int cat = getColorHist(cloud_point);
		if(cat != -1)
			colors[cat] ++;
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

int track_3d::getColorHist(pcl::PointXYZRGB cloud_point){
	double R, G, B;
	double hue = 0;
	int hue_cat = -1;
	
	R = 0; G = 0; B = 0;
	R += cloud_point.r; G += cloud_point.g; B += cloud_point.b; 
	
	double Cmax = maximum(R, G, B);
	double Cmin = minimum(R, G, B);
	double delta = Cmax - Cmin;
	if(R > G && R > B){
		hue = 60*((double)((G-B)/delta));
	}else if(G > R && G > B){
		hue = 60*((double)((B-R)/delta)+2);
	}else if(B > R && B > G){
		hue = 60*((double)((R-G)/delta)+4);
	}
	if(hue < 0)
		hue += 360;
	
	if(hue >= 0 && hue <= 60){
		hue_cat = 0;
	}else if(hue >= 61 && hue <= 160){
		hue_cat = 1;
	}else if(hue >= 161 && hue <= 250){
		hue_cat = 2;
	}else if(hue >= 251 && hue <= 310){
		hue_cat = 3;
	}else if(hue >= 311 && hue <= 360){
		hue_cat = 4;
	}
	
	return hue_cat;
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

string track_3d::getAllHues(){
	string hues = "";
	
	int index = 1;
	for(auto color: colors){
		if(color > 100){
			hues = hues + to_string(index) + " ";
		}
		index ++;
	}
	return hues;
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



/* Function maximum definition */
/* x, y and z are parameters */
double track_3d::maximum(double x, double y, double z) {
	double max = x; /* assume x is the largest */

	if (y > max) { /* if y is larger than max, assign y to max */
		max = y;
	} /* end if */

	if (z > max) { /* if z is larger than max, assign z to max */
		max = z;
	} /* end if */

	return max; /* max is the largest value */
} /* end function maximum */

/* Function minimum definition */
/* x, y and z are parameters */
double track_3d::minimum(double x, double y, double z) {
	double min = x; /* assume x is the smallest */

	if (y < min) { /* if y is smaller than min, assign y to min */
		min = y;
	} /* end if */

	if (z < min) { /* if z is smaller than min, assign z to min */
		min = z;
	} /* end if */ 

	return min; /* min is the smallest value */
} /* end function minimum */

double track_3d::gaussian(double in, double mean, double std)
{
      return (1./(std*sqrt(2*M_PI)))*exp(-0.5*(pow(in-mean, 2)/(pow(std,2))));
}

track_3d::~track_3d(void)
{
}
