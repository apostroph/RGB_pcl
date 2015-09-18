/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/
#define DESIRED_FPS 30

#include "Recognition.h"


rgb_pcl::rgb_pcl(int argc, char** argv):
target_frame(""), ongoing_action(false)
{    
	//namedWindow("Vision and states", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	n = ros::NodeHandle("~");
	
	//Link the escape comd to the sign out method
	signal(SIGINT, &rgb_pcl::sigintHandler);
	
	// Create a ROS subscriber for the input point cloud
	sub = n.subscribe ("/head_mount_kinect/depth_registered/points", 1, &rgb_pcl::cloud_cb, this);
	action_feedback = n.subscribe ("/action_state_feedback", 1, &rgb_pcl::fromBackend, this);
	
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



void rgb_pcl::fromBackend(const std_msgs::String msg){
	string input = msg.data.c_str();
	if(input.compare("action_ended") == 0){
		ongoing_action = false;
	}else{
		ongoing_action = true;
	}
}

//Method called everytime a pointcloud is received
void rgb_pcl::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
	
	if(!ongoing_action){
		// Container for original & filtered data
		if(target_frame.find(base_frame) == std::string::npos){
			getTransformCloud(input, *input);
		}
		sensor_msgs::PointCloud2 in = *input;
		sensor_msgs::PointCloud2 out;
		pcl_ros::transformPointCloud(target_frame, net_transform, in, out);
		
		pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
		pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
		pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
					  
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
							cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), 
							cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
							
		Mat displayImage = cv::Mat(Size(640, 480), CV_8UC3);
		displayImage = Scalar(120);
		// Convert to PCL data type
		pcl_conversions::toPCL(out, *cloud);
		
		// Perform the actual filtering
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
		sor.setInputCloud (cloudPtr);
		sor.setLeafSize (0.005, 0.005, 0.005);
		sor.filter (*cloud_filtered_blob);
		
		pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

		ModelCoefficientsPtr coefficients (new pcl::ModelCoefficients);
		
		PointCloudPtr plane_points(new PointCloud), point_points_2d_hull(new PointCloud);
			
		std::vector<PointCloudPtr> object_clouds;
		pcl::PointCloud<pcl::PointXYZRGB> combinedCloud;
		
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
		
		interpretTableScene(cloud_filtered, coefficients, plane_points, point_points_2d_hull, object_clouds);
		
		int c = 0;
		int ID_object = -1;
		
		for(auto cloudCluster: object_clouds){
	// 		get_cloud_matching(cloudCluster); //histogram matching
		
			ID_object = c;
			combinedCloud += *cloudCluster;
			combinedCloud.header = cloud_filtered->header;
			c++;
		}
		
		getTracker(object_clouds);
		
		stateDetection();
		
		sensor_msgs::PointCloud2 output;
		
		if((int)object_clouds.size() >= ID_object && ID_object >= 0){
			pcl::toROSMsg(combinedCloud, output);
			// Publish the data
			pub.publish (output);
		}
	}
	
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
}

void rgb_pcl::getTracker(std::vector<PointCloudPtr> object_clouds){
	
	
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
		}
	}
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

void rgb_pcl::stateDetection(){
	RGB_pcl::States message;
	ROS_INFO("Scene recognition");
	for(auto tracker: trackerList){
		if(!tracker.isGone()){
			string state;
			double volume = tracker.getLength() * tracker.getWidth() * tracker.getHeight();
			if(volume > 0.002){
				state = state + "big ";
			}else if(volume > 0.00045){
				state = state + "medium ";
			}else{
				state = state + "small ";
			}
			
			if(tracker.getAllHues().size() != 0){
				state = state + tracker.getAllHues();
				
				double x_c = tracker.getPosition().x;
				double y_c = tracker.getPosition().y;
				
				double back[2] = {0.82, 0.04};
				double front[2] = {0.60, 0.042};
				double left[2] = {0.686, 0.187};
				double right[2] = {0.67, -0.118};
				double sen = 0.04;
				
				if(x_c < 0.8){
					if(x_c > (back[0]-sen) && x_c < (back[0]+sen) && y_c > (back[1]-sen) && y_c < (back[1]+sen)){
						state = state + "back ";
					}else if(x_c > (right[0]-sen) && x_c < (right[0]+sen) && y_c > (right[1]-sen) && y_c < (right[1]+sen)){
						state = state + "right ";
					}else if(x_c > (left[0]-sen) && x_c < (left[0]+sen) && y_c > (left[1]-sen) && y_c < (left[1]+sen)){
						state = state + "left ";
					}else if(x_c > (front[0]-sen) && x_c < (front[0]+sen) && y_c > (front[1]-sen) && y_c < (front[1]+sen)){
						state = state + "front ";
					}
				}else{
					state = state + "out ";
				}
				
				message.states.push_back(state);
				message.x.push_back(tracker.getGraspPosition().x);
				message.y.push_back(tracker.getGraspPosition().y);
				message.z.push_back(tracker.getGraspPosition().z);
				
				sensor_msgs::PointCloud2 output;
				pcl::toROSMsg(tracker.getPointCloud(), output);
				message.clusters.push_back(output);
				cout<<state<<":: ";
// 				cout<<volume<<" : ";
// 				cout<<x_c<<" : "<<y_c<<endl;
			}
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