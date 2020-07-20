#include "ros/ros.h"
#include "planner/polyfit.h"
#include "planner/polyfit.c"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "cmath"
#include "tuple"

#include <iostream>
#include <string>
#include <stdlib.h>

#define GLOBAL_PATH_FILE "/home/hyeonbeen/path.txt"

#define DIST_OBS_HP 3.0 // range to detect obstacles
#define CLUSTER_HP 1.5 // distance to clustering between points

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZI VPoint;

class OdomDouble {
	private:
		double x, y, z;

	public:
		OdomDouble(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}

		// getters
		double getX() {
			return this->x;
		}

		double getY() {
			return this->y;
		}

		double getZ() {
			return this->z;
		}
};

class LanePoint{
	public:
		float x, y;
		int layer;

		LanePoint() { }

		LanePoint(float x, float y, int layer) {
			this->x = x;
			this->y = y;
			this->layer = layer;
		}
};


class Point {
    public:
        float x,y,z,intensity;
        int clusterID, layer;

	    Point() {};

        Point(float x, float y, float z, float intensity, int layer) {
		    this->x = x;
		    this->y = y;
		    this->z = z;
		    this->intensity = intensity;
		    this->layer = layer;
	    };
};

class Planner{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, point_pub_;
    ros::Subscriber sub_, obstacle_sub_;

    double OFFSET_X = 0;
    double OFFSET_Y = 0;
    double odom_x, odom_y, odom_z;
    double pose_x, pose_y, pose_z, pose_w;
	double roll, pitch, yaw, yaw_d;

	vector<OdomDouble> global_path_;
	vector<OdomDouble> local_path_;

public:
    void initSetup();
    void setPlan();
	vector<OdomDouble> loadGlobalPath();
	void printGlobalPath(vector<OdomDouble>);
	void obstacleCallback(const pcl::PointCloud<VPoint>::ConstPtr& obstacles);
	void makeLocalPath(vector<vector<VPoint>> result_points);
};
