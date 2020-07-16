#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "velodyne_pointcloud/point_types.h"

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"

#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>


#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "cmath"

using namespace std;
typedef pcl::PointXYZI PointType;

class Cluster {
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, point_pub_, acker_pub_;
    ros::Subscriber sub_, odom_sub_, point_sub_;

public:
    void initSetup();
	sensor_msgs::PointCloud2 clusterCallback(const sensor_msgs::PointCloud2ConstPtr &input);
};
