#include "planner.h"

void Planner::initSetup(){
    odom_sub_ = nh_.subscribe("/odom", 10, &Planner::odomCallback, this);
    point_sub_ = nh_.subscribe("/velodyne_points", 10, &Planner::pointCallback, this);    
}

void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs){
    odom_x = odom_msgs->pose.pose.position.x;
    odom_y = odom_msgs->pose.pose.position.y;
    odom_z = odom_msgs->pose.pose.position.z;
    tf::Quaternion q(
		odom_msgs->pose.pose.orientation.x,
		odom_msgs->pose.pose.orientation.y,
		odom_msgs->pose.pose.orientation.z,
		odom_msgs->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw, yaw_d;
	m.getRPY(roll, pitch, yaw);

	yaw_d = yaw*180/M_PI;
    
}
void Planner::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
//clustering step -> recognition obstacle and cacluate distance
//if in specific distance, then remake set plan like using offset 
}

void Planner::calDistance(){}

void Planner::setPlan(){
}

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    Planner pl;
    pl.initSetup();
}