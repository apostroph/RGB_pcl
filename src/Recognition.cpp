/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define PUBLISH_CLOUDS 1

#define PR2 1
#define DISPLAY 0

#define DESIRED_FPS 30

#include "Recognition.h"


rgb_pcl::rgb_pcl(int argc, char** argv):
imgCount(0), target_frame("")
{    
	//namedWindow("Vision and states", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	n = ros::NodeHandle("~");
	
	//Link the escape comd to the sign out method
	signal(SIGINT, &rgb_pcl::sigintHandler);
	
	// Create a ROS subscriber for the input point cloud
#if PR2
	sub = n.subscribe ("/head_mount_kinect/depth_registered/points", 1, &rgb_pcl::cloud_cb, this);
#endif
#if !PR2
	sub = n.subscribe ("/kinect1/depth_registered/points", 1, &rgb_pcl::cloud_cb, this);
#endif
	// Create a ROS publisher for the output point cloud
	pub = n.advertise<sensor_msgs::PointCloud2> ("point_cloud_jimmy", 1);

	// Create a ROS publisher for observed states list
	pubAct = n.advertise<RGB_pcl::States> ("actions", 1);
	
	// Create a ROS publisher for observed crop box marker
	marker_publisher = n.advertise<visualization_msgs::Marker>("world_object_marker", 0);
	table_publisher = n.advertise<visualization_msgs::Marker>("world_table_marker", 0);
	
// 	loadHistogram();
	

	base_frame = "base_link";

	ROS_INFO("Ready start the rgb_pcl");
}

void rgb_pcl::sigintHandler(int sig)
{
	ROS_INFO("rgb_pcl module shutdown");
	ros::shutdown();
	
	return;
}

void rgb_pcl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
	
	// Container for original & filtered data
#if PR2
	if(target_frame.find(base_frame) == std::string::npos){
		getTransformCloud(input, *input);
	}
	sensor_msgs::PointCloud2 in = *input;
	sensor_msgs::PointCloud2 out;
	pcl_ros::transformPointCloud(target_frame, net_transform, in, out);
#endif
	
// 	ROS_INFO("Cloud acquired...");
	
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
				  
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
						 cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
						 cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
						 
	Mat displayImage = cv::Mat(Size(640, 480), CV_8UC3);
	displayImage = Scalar(120);

	// Convert to PCL data type
#if PR2
	pcl_conversions::toPCL(out, *cloud);
#endif
#if !PR2
	pcl_conversions::toPCL(*input, *cloud);
#endif
// 	ROS_INFO("\t=>Cloud rotated...");

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.005, 0.005, 0.005);
	sor.filter (*cloud_filtered_blob);
	
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
	
// 	ROS_INFO("\t=>Cloud converted...");	

// 	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	ModelCoefficientsPtr coefficients (new pcl::ModelCoefficients);
	
	PointCloudPtr plane_points(new PointCloud), point_points_2d_hull(new PointCloud);
		
	std::vector<PointCloudPtr> object_clouds;
	pcl::PointCloud<pcl::PointXYZRGB> combinedCloud;
	
// 	for(int count = 0; count < cloud_filtered->points.size(); count++){
// 		if(cloud_filtered->points[count].z < 0.2){
// 			cloud_filtered->points.erase(cloud_filtered->points.begin()+count);
// 			count--;
// 		}
// 	}
#if PR2
	make_crop_box_marker(marker_publisher, base_frame, 0, 0.2, -1, 0.2, 1.3, 2, 1.3);
// 	Define your cube with two points in space: 
	Eigen::Vector4f minPoint; 
	minPoint[0]=0.2;  // define minimum point x 
	minPoint[1]=-1;  // define minimum point y 
	minPoint[2]=0.2;  // define minimum point z 
	Eigen::Vector4f maxPoint; 
	maxPoint[0]=1.5;  // define max point x 
	maxPoint[1]=1;  // define max point y 
	maxPoint[2]=1.5;  // define max point z 

	pcl::CropBox<pcl::PointXYZRGB> cropFilter; 
	cropFilter.setInputCloud (cloud_filtered); 
	cropFilter.setMin(minPoint); 
	cropFilter.setMax(maxPoint); 

   	cropFilter.filter (*cloud_filtered); 
#endif
	
// 	ROS_INFO("\t=>Cloud croped...");
	
#if !PR2
	//Rotate the point cloud
	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI; // The angle of rotation in radians

	// Define a translation of 0 meters on the x axis
	transform_1.translation() << 0.0, 0.0, 1.0;

	// The same rotation matrix as before; tetha radians arround X axis
	transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

	// Executing the transformation
	pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_1);
#endif
	
	interpretTableScene(cloud_filtered, coefficients, plane_points, point_points_2d_hull, object_clouds);
// 	ROS_INFO("\t=>Cloud interpreted...");
// 	pcl::PointXYZRGB min_pt, max_pt;
// 	pcl::getMinMax3D(*plane_points, min_pt, max_pt);
// 	
// 	make_crop_box_marker(table_publisher, base_frame, 1, min_pt.y, min_pt.z, min_pt.z, max_pt.y, max_pt.x, min_pt.z+5);
	
	int c = 0;
#if PUBLISH_CLOUDS
	int ID_object = -1;
#endif
	for(auto cloudCluster: object_clouds){
// 		get_cloud_matching(cloudCluster); //histogram matching
	
#if PUBLISH_CLOUDS
		ID_object = c;
#endif
		combinedCloud += *cloudCluster;
		combinedCloud.header = cloud_filtered->header;
		c++;
	}
	
#if DISPLAY
	drawPointCloud(combinedCloud, displayImage);
#endif
	
	getTracker(object_clouds, displayImage);
	
	stateDetection();
// 	ROS_INFO("\t=>Cloud analysed...");
	
#if PUBLISH_CLOUDS
	
	sensor_msgs::PointCloud2 output;
	
	if(object_clouds.size() >= ID_object && ID_object >= 0){
		pcl::toROSMsg(combinedCloud, output);
		// Publish the data
		pub.publish (output);
	}
	
#endif
	
	end = ros::Time::now();
	std::stringstream ss;
	ss <<(end-begin);
	string s_FPS = ss.str();
#if DISPLAY
	cv::putText(displayImage, "FPS: "+to_string((int)1/(stof(s_FPS))) + "   Desired: "+to_string(DESIRED_FPS), cv::Point(10, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
	imshow("RGB", displayImage);
#endif
	waitKey(1);

	begin = ros::Time::now();
// 	ROS_INFO("\t=>Cloud treatment over...");
	
}

void rgb_pcl::getTransformCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr, const sensor_msgs::PointCloud2& upright_cloud_msg){
	tf::TransformListener tf_listener;
	//Rotate the point cloud
	tf::StampedTransform transform;
	
	// transform point cloud
	ROS_INFO("Waiting for transform -> %s",(*cloud_msg_ptr).header.frame_id.c_str());
	tf_listener.waitForTransform("/"+base_frame, (*cloud_msg_ptr).header.frame_id, ros::Time(0), ros::Duration(3.0));
	ROS_INFO("... done waiting.");
	tf_listener.lookupTransform("/"+base_frame, (*cloud_msg_ptr).header.frame_id, ros::Time(0), transform);
	ROS_INFO("Transforming point cloud...");
	
	target_frame = "/"+base_frame;
	net_transform = transform;
// 	sensor_msgs::PointCloud2 in = *cloud_msg_ptr;
// 	sensor_msgs::PointCloud2 out = upright_cloud_msg;
	
// 	pcl_ros::transformPointCloud(target_frame, net_transform, in, out);
	
// 	pcl::fromROSMsg(*upright_cloud_msg, *point_cloud_);
//     //    pcl::fromROSMsg(upright_cloud_msg, *point_cloud_rgb_);
// 	pcl::fromROSMsg(*cloud_msg_ptr, *point_cloud_rgb_);
}

void rgb_pcl::getTracker(std::vector<PointCloudPtr> object_clouds, Mat displayImage){
	
	
	//Assign tracker to the clouds
	for(unsigned int count1 = 0; count1 < trackerList.size(); count1++){
		cv::Point3d position(0, 0, 0);
		double maxScore = 0;
		int id_cloud = -1;
		cv::Point3d pos_tracker;
		int size_tracker;
		double hue_tracker;
		
		for(unsigned int count2 = 0; count2 < object_clouds.size(); count2 ++){
			int size = object_clouds[count2]->points.size();
			
			cv::Point3d position(0, 0, 0);
			double hue = 0;
			
			getCloudFeatures(position, hue, object_clouds[count2]);
			
			double score = trackerList[count1].isRecognized(position, 0, size);
			if(score > maxScore){
				maxScore = score;
				id_cloud = count2;
				pos_tracker = position;
				size_tracker = size;
				hue_tracker = hue;
			}
		}
		if(id_cloud != -1){
			trackerList[count1].updateTracker(pos_tracker, hue_tracker, size_tracker, object_clouds[id_cloud]);
			object_clouds.erase(object_clouds.begin()+id_cloud);
		}
	}
// 	cout<<trackerList.size()<<endl;
	//Create new tracker for the remaining clouds
	for(auto cloudCluster: object_clouds){
		int size = object_clouds[0]->points.size();
		
		cv::Point3d position(0, 0, 0);
		double hue = 0;
		
		getCloudFeatures(position, hue, cloudCluster);
		
		track_3d newTracker(position, hue, size);
		trackerList.push_back(newTracker);
	}
	
	//Delete lost tracker and display tracked clouds
	for(unsigned int count = 0; count < trackerList.size(); count ++){
		trackerList[count].step();
		if(!trackerList[count].isAlive()){
			trackerList.erase(trackerList.begin()+count);
			count --;
		}else if(trackerList[count].isFound() && !trackerList[count].isGone()){
#if DISPLAY
			drawTrackers(displayImage, trackerList[count], to_string(count));
#endif
		}
	}
}

void rgb_pcl::drawTrackers(Mat &img, track_3d input_tracker, string name){
	double x, y;
	double width, height;
	
	x = input_tracker.getPosition().x;
	y = input_tracker.getPosition().y;
	width = input_tracker.getLength();
	height = input_tracker.getWidth();
	
	cv::Point tracker_center;
	cv::Point tracker_size;
	
	//transform center position from kinect space to 640*480 Mat
// 	tracker_center.x = (x) * 640;
// 	tracker_center.y = (-y+0.5) * 480;
	
	tracker_center.x = (-y)*320 + 320;
	tracker_center.y = 480-(x)*320;
	
	//transform tracker size from kinect space to 640*480 Mat
	tracker_size.x = abs(height) * 320;
	tracker_size.y = abs(width) * 320;
	
// 	cout<<tracker_size.x<<" :: "<<tracker_size.y<<endl;
	
	
	cv::Point p1(tracker_center.x-(tracker_size.x/2.), tracker_center.y-(tracker_size.y/2.));
	cv::Point p2(tracker_center.x+(tracker_size.x/2.), tracker_center.y+(tracker_size.y/2.));
	
	rectangle(img, p1, p2, Scalar(0 , 255, 0), 2);
	
	cv::putText(img, name, cv::Point(tracker_center.x, tracker_center.y-10-(tracker_size.y/2.)), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
}

void rgb_pcl::getCloudFeatures(cv::Point3d &position, double &hue, PointCloudPtr cloudCluster){
	double R, G, B;
	double size = cloudCluster->points.size();
	
	R = 0; G = 0; B = 0;
	double maxZ = 0;
	for(auto cloud_point: cloudCluster->points){
		position += Point3d(cloud_point.x, cloud_point.y, cloud_point.z);
		R += cloud_point.r; G += cloud_point.g; B += cloud_point.b; 
		if(cloud_point.z > maxZ){
			maxZ = cloud_point.z; //We want to grab the object at the highest z
		}
	}
	position.x /= size; position.y /= size; position.z = maxZ;
	R /= size; G /= size; B /= size; 
	
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
}

/* Function maximum definition */
/* x, y and z are parameters */
double rgb_pcl::maximum(double x, double y, double z) {
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
double rgb_pcl::minimum(double x, double y, double z) {
	double min = x; /* assume x is the smallest */

	if (y < min) { /* if y is smaller than min, assign y to min */
		min = y;
	} /* end if */

	if (z < min) { /* if z is smaller than min, assign z to min */
		min = z;
	} /* end if */

	return min; /* min is the smallest value */
} /* end function minimum */

void rgb_pcl::drawPointCloud(pcl::PointCloud<pcl::PointXYZRGB> combinedCloud, Mat displayImage){
	
	for(auto pointIT: combinedCloud.points){
		double x, y;
		x = (-pointIT.y)*320 + 320;
		y = 480-(pointIT.x)*400;
		if(x < 640 && x > 0 && y < 480 && y > 0){
			circle(displayImage, cv::Point(x, y), 3, Scalar(pointIT.b, pointIT.g, pointIT.r), -1);
		}
	}
}

void rgb_pcl::stateDetection(){
	RGB_pcl::States message;
	cout<<"RGB: ";
	for(auto tracker: trackerList){
		if(!tracker.isGone()){
			string state;
			double volume = tracker.getLength() * tracker.getWidth() * tracker.getHeight();
			if(volume > 0.002){
				state = state + "big ";
			}else if(volume > 0.0006){
				state = state + "medium ";
			}else{
				state = state + "small ";
			}

			int hue_cat = tracker.getHue()/36;
			state = state + to_string(hue_cat) + " ";
			
			double x_c = tracker.getPosition().x;
			double y_c = tracker.getPosition().y;
			
			if(x_c > 0.68 && x_c < 0.76 && y_c > 0.01 && y_c < 0.09){
				state = state + "in ";
			}else if(x_c > 0.66 && x_c < 0.74 && y_c > -0.17 && y_c < -0.09){
				state = state + "right ";
			}else if(x_c > 0.66 && x_c < 0.74 && y_c > 0.14 && y_c < 0.22){
				state = state + "left ";
			}else if(x_c > 0.55 && x_c < 0.64 && y_c > 0.0 && y_c < 0.08){
				state = state + "front ";
			}
			
			message.states.push_back(state);
			message.x.push_back(tracker.getGraspPosition().x);
			message.y.push_back(tracker.getGraspPosition().y);
			message.z.push_back(tracker.getGraspPosition().z);
			
			sensor_msgs::PointCloud2 output;
			pcl::toROSMsg(tracker.getPointCloud(), output);
			message.clusters.push_back(output);
			
			cout<<state<<"  ::  ";
// 			cout<<x_c<<" : "<<y_c<<endl;
		}
		
	}
	cout<<endl;
	pubAct.publish (message);
}
    
/* Called periodically every getPeriod() seconds */
bool rgb_pcl::loop() {
    
  //loop rate at 30FPS max
    ros::Rate loop_rate(DESIRED_FPS);
    
    while(ros::ok()){
      
	ros::spinOnce();
	
	loop_rate.sleep();
	waitKey(1);
    }
    ros::shutdown();
	
    return true;
}

void rgb_pcl::make_crop_box_marker(ros::Publisher publisher, std::string frame, int idx, double x_start, double y_start, double z_start, double x_size, double y_size, double z_size) {

	visualization_msgs::Marker marker;
	marker.header.frame_id = frame.c_str();
	marker.header.stamp = ros::Time(0);
	marker.ns = "crop_box";
	marker.id = idx;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x_start + 0.5 * x_size;
	marker.pose.position.y = y_start + 0.5 * y_size;
	marker.pose.position.z = z_start + 0.5 * z_size;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = x_size;
	marker.scale.y = y_size;
	marker.scale.z = z_size;
	std_msgs::ColorRGBA color;
	marker.color.a = 0.2;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	//ROS_INFO("published crop box marker");
	publisher.publish(marker);
}


rgb_pcl::~rgb_pcl(){
	cout<<"Shutdown"<<endl;
}

// not used now


void rgb_pcl::loadHistogram(){
	for(int c = 1; c <= 2; c++){
		string src ="/home/jimmy/programs/ros/catkin_ws/src/RGB_pcl/markers/mark_"+to_string(c)+".jpg";
		cout<<src<<endl;
		Mat marker = imread("/home/jimmy/programs/ros/catkin_ws/src/RGB_pcl/markers/mark_"+to_string(c)+".jpg", 1);	/// Using 50 bins for hue and 60 for saturation
		Mat marker_hsv;
		
		cvtColor( marker, marker_hsv, COLOR_BGR2HSV );
		
		int h_bins = 50; int s_bins = 60;
		int histSize[] = { h_bins, s_bins };

		// hue varies from 0 to 179, saturation from 0 to 255
		float h_ranges[] = { 0, 180 };
		float s_ranges[] = { 20, 256 };
		float v_ranges[] = { 20, 256 };

		const float* ranges[] = { h_ranges, s_ranges, v_ranges };

		// Use the o-th and 1-st channels
		int channels[] = { 0, 1, 2 };
		
		/// Histograms
		MatND hist;
		
		calcHist( &marker_hsv, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
		normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
		
		histogram_list.push_back(hist);
	}
}


void rgb_pcl::get_cloud_matching(PointCloudPtr cloud){
	
	
	Mat result;
	
	double limit_x[2] = {DBL_MIN, DBL_MAX};
	double limit_y[2] = {DBL_MIN, DBL_MAX};
	
	for(auto cloud_point: cloud->points){
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
	}
	
	int height = (abs(limit_x[1]-limit_x[0]))*640+1;
	int width = (abs(limit_y[1]-limit_y[0]))*480+1;
	
	Mat object(width, height, CV_8UC3, Scalar(0));
	
	for(auto pointIT: cloud->points){
		double x, y;
		x = abs(pointIT.x-limit_x[0])*640+1;
		y = abs(pointIT.y-limit_y[0])*480+1;
		if(x < 640 && x > 0 && y < 480 && y > 0){
			circle(object, cv::Point(x, y), 2, Scalar(pointIT.b, pointIT.g, pointIT.r), -1);
		}
	}
	
	int c = 1;
	double max = 15;
	int index = -1;
	for(auto hist: histogram_list){
		double value = MatchingMethod(object, hist, result, 1);
		if(value > max){
			max = value;
			index = c;
		}
		c++;
	}	
	
	if(max > 15){
		cout<<" >> matched with histogram "<<index<<" and value = "<<max<<endl; 
	}
}

double rgb_pcl::MatchingMethod(Mat img, MatND hist_templ, Mat &result, int match_method)
{
	Mat hsv_base;
	
	cvtColor( img, hsv_base, COLOR_BGR2HSV );
	
	/// Using 50 bins for hue and 60 for saturation
	int h_bins = 50; int s_bins = 60;
	int histSize[] = { h_bins, s_bins };

	// hue varies from 0 to 179, saturation from 0 to 255
	float h_ranges[] = { 0, 180 };
	float s_ranges[] = { 20, 256 };
	float v_ranges[] = { 20, 256 };

	const float* ranges[] = { h_ranges, s_ranges, v_ranges };

	// Use the o-th and 1-st channels
	int channels[] = { 0, 1, 2 };
	
	/// Histograms
	MatND hist_base;

	/// Calculate the histograms for the HSV images
	calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
	normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

	/// Apply the histogram comparison methods

	double base_base = compareHist( hist_base, hist_templ, match_method );
	
	return base_base;
  
}

