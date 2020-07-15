#include "planner/planner_.h"

void Planner::initSetup(){
    point_sub_ = nh_.subscribe("/velodyne_points", 10, &Planner::pointCallback, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);
}

void Planner::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input){
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>), cloud_filterd (new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*input, *cloud);

    /*pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType>());
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, 0.0)));  //eg. z축으로 0.00보다 큰값(GT:Greater Than)
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, 0.8)));  //eg. z축으로 0.08보다 작은값(LT:Less Than)
    pcl::ConditionalRemoval<PointType> condrem;
    condrem.setInputCloud (cloud);        //입력 
    condrem.setCondition (range_cond);    //조건 설정  
    condrem.setKeepOrganized(false);       //
    condrem.filter (*cloud_filterd);     //필터 적용 
    */

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 
   
    pcl::SACSegmentation<PointType> seg;
    
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);
    seg.setMaxIterations(100);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_filterd);
    
    point_pub_.publish(cloud_filterd);    

}

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    Planner pn;
    pn.initSetup();
    ros::spin();
}