/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef _rgb_pcl_H_
#define _rgb_pcl_H_

#include <string>

#include <iostream>
#include <iomanip>

#include <signal.h>
#include "std_msgs/String.h"

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "ros/ros.h"
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>

#include "pcl_utils/pcl_utils.h"

#include "track_utils/track_3d.h"
#include "RGB_pcl/States.h"

using namespace std;
using namespace cv;
using namespace picard;

class rgb_pcl{

public:
    /** 
     * document your methods too.
     */
    rgb_pcl(int argc, char** argv);
    ~rgb_pcl();
    
    bool loop();

private:
  //ROS quit handler
  static void sigintHandler(int sig);
  
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber action_feedback;
  
  ros::Publisher pub;
  ros::Publisher pubAct;
  ros::Publisher marker_publisher;
  ros::Publisher table_publisher;
  ros::ServiceServer service;
  
  string base_frame;
  
  tf::Transform net_transform;
  std::string target_frame;
  
  ros::Time begin, end;
  bool ongoing_action;
  
  vector<track_3d> trackerList;
  vector<MatND> histogram_list;
  
  void fromBackend(const std_msgs::String msg);
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
  void getTransformCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr, const sensor_msgs::PointCloud2& upright_cloud_msg);
  
  void getCloudFeatures(cv::Point3d &position, double &hue, PointCloudPtr object_clouds);
  void getTracker(std::vector<PointCloudPtr> object_clouds);
  
  void stateDetection();
  
  double maximum(double x, double y, double z);
  double minimum(double x, double y, double z);
  
  void make_crop_box_marker(ros::Publisher publisher, std::string frame, int idx, double x_start, double y_start, double z_start, double x_size, double y_size, double z_size);
  
  
};


#endif // __rgb_pcl_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>

// #include <cv.h>
// #include "opencv2/highgui/highgui.hpp"

// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/surface/concave_hull.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl/common/centroid.h>
// #include <pcl/common/common.h>
// #include <pcl/common/transforms.h>
// #include <pcl/filters/filter.h>
// #include <pcl/filters/crop_box.h>
// #include <pcl/PointIndices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/extract_indices.h>
