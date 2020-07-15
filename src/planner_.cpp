#include "planner/planner_.h"

void Planner::initSetup(){
    point_sub_ = nh_.subscribe("/velodyne_points", 10, &Planner::pointCallback, this);
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 10);
}

void Planner::pointCallback(const sensor_msgs::PointCloud2ConstPtr &input){
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>), cloud_filtered (new pcl::PointCloud<PointType>);

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

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits(-4, 4);
    pass.filter(*cloud);
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-4, 4);
    pass.filter(*cloud);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 
    point_pub_.publish(cloud);    

    pcl::SACSegmentation<PointType> seg;
    
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);
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
    extract.filter(*cloud_filtered);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree -> setInputCloud(cloud_filtered);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.3); //20cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    
    pcl::PointCloud<PointType> Result_cloud;
    int j = 0;
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            PointType pt2;
            pt2.x = cloud_filtered->points[*pit].x, pt2.y = cloud_filtered->points[*pit].y, pt2.z = cloud_filtered->points[*pit].z;
            pt2.intensity = cloud_filtered->points[*pit].intensity;

            Result_cloud.push_back(pt2);
        }
        j++; 
    }
    Result_cloud.header.frame_id = "velodyne";
    pub_.publish(Result_cloud);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "Planner");
    Planner pn;
    pn.initSetup();
    ros::spin();
}
