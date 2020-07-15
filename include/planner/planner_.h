#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/conditional_removal.h"
#include <pcl/filters/extract_indices.h>

#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "cmath"

using namespace std;
typedef pcl::PointXYZI PointType;

class Planner{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, point_pub_, acker_pub_;
    ros::Subscriber sub_, odom_sub_, point_sub_;

public:
    void initSetup();
    void pointCallback(const sensor_msgs::PointCloud2ConstPtr &input);
};
