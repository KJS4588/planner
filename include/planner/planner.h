#include "ros/ros.h"
#include "planner/polyfit.h"
#include "planner/polyfit.c"
#include "planner/euclidean_cluster.cpp"

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

#define GLOBAL_PATH_FILE "/home/<change>/catkin_ws/src/global_path_generator/src/path.txt"

using namespace std;
typedef pcl::PointXYZI PointType;

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
			return this->Y;
		}

		double getZ() {
			return this->Z;
		}
}

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
};
