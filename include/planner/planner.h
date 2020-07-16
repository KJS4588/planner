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

using namespace std;

typedef pcl::PointXYZI PointType;

class OdomDouble {
	private:
		double x, y, z;
		double o_x, o_y, o_z, o_w;

	public:
		OdomDouble(double x, double y, double z, double o_x, double o_y, double o_z, double o_w) {
			this->x = x;
			this->y = y;
			this->z = z;
			this->o_x = o_x;
			this->o_y = o_y;
			this->o_z = o_z;
			this->o_w = o_w;
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

		double getOX() {
			return this->o_x;
		}

		double getOY() {
			return this->o_y;
		}

		double getOZ() {
			return this->o_z;
		}

		double getOW() {
			return this->o_w;
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
    ros::Subscriber sub_, odom_sub_, point_sub_;
    double OFFSET_X = 0;
    double OFFSET_Y = 0;
    double odom_x, odom_y, odom_z;
    double pose_x, pose_y, pose_z, pose_w;
	double roll, pitch, yaw, yaw_d;

	vector<float> left_poly_;
	vector<float> right_poly_;
    vector<nav_msgs::Odometry> path_;

	vector<OdomDouble> global_path_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);
    double calcDistance(double * coef, pcl::PointXYZ object_point);
    void setPlan(double* left_coef , double* right_coef, sensor_msgs::PointCloud2 objects);
    tuple<double *, double *> getLine(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr &cloud_XYZIR);
	void visualize(vector<LanePoint> left_lane, vector<LanePoint> right_lane, vector<geometry_msgs::Point> waypoint);
	pcl::PointXYZ getClosestObject(sensor_msgs::PointCloud2 object);
	vector<OdomDouble> loadGlobalPath();
	void printGlobalPath(vector<OdomDouble>);
};
