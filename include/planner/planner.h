#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "cmath"
#include "math.h"
#include "tuple"

#include <iostream>
#include <string>
#include <stdlib.h>

#include "planner/OdomDouble.h"

#define GLOBAL_PATH_FILE "/home/hyeonbeen/path.txt"
#define NEW_GLOBAL_PATH_FILE "/home/hyeonbeen/new_path.txt"

#define DIST_OBS_HP 3.0 // range to detect obstacles
#define CLUSTER_HP 1.5 // distance to clustering between points

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZI VPoint;

class Planner{
private:
    ros::NodeHandle nh_;

    ros::Publisher point_pub_;
    ros::Publisher marker_pub_;

    ros::Subscriber aligned_sub_;

    double OFFSET_X = 0;
    double OFFSET_Y = 0;
    double odom_x, odom_y, odom_z;
    double pose_x, pose_y, pose_z, pose_w;
	double roll, pitch, yaw, yaw_d;

	vector<OdomDouble> global_path_;
	vector<OdomDouble> local_path_;

    int obs_detect_flag_ = 0;

public:
    void initSetup();

	void alignedCallback(const sensor_msgs::PointCloud2ConstPtr& aligned_points);

    void setPlan();
	void loadGlobalPath();
	void calcDistance();
	vector<double> getLinearValues();
	int getClosestPointIndex(geometry_msgs::Point p);
	void visualize(vector<OdomDouble> global_path);
	double getDist(geometry_msgs::Point point_1, OdomDouble p );
    void savePath();
};
