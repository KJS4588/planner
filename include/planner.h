#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

using namespace std;
typedef pcl::PointXYZI PointType;

class Planner{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_, odom_sub_, point_sub_;
    double OFFSET_X = 0;
    double OFFSET_Y = 0;
    double odom_x, odom_y, odom_z;
public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);
    void clustering();
    void calDistance();
    void setPlan();
};
