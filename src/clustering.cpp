#include "planner/clustering.h"

vector<geometry_msgs::Point> Cluster::cluster(const sensor_msgs::PointCloud2ConstPtr &input){
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>), cloud_filterd (new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*input, *cloud);

    /*pcl::ConditionAnd<PointType>::Ptr range_cond (new pcl::ConditionAnd<PointType>());
    range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, 0.0)));  //eg. z축으로 0.00보다 큰값(GT:Greater Than) range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, 0.8)));  //eg. z축으로 0.08보다 작은값(LT:Less Than)
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
	
	pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1, 1000);
    pass.filter(*cloud);


    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create a pcl object to hold the ransac filtered object
    pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType>()); 
    
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
    extract.filter(*cloud_filterd);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree -> setInputCloud(cloud_filterd);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.3); //20cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filterd);
    ec.extract(cluster_indices);


    pcl::PointCloud<PointType> cluster_cloud1, cluster_cloud2, cluster_cloud3, Result_cloud;
    cout << "Number of clusters is equal to " << cluster_indices.size() << endl;
    int j = 0;

    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){
        if(it == cluster_indices.begin()){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);

            	cluster_cloud1.push_back(pt2);}
		}
        
		if(it == cluster_indices.begin()+1){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud2.push_back(pt2);}
		}
		
        if(it == cluster_indices.begin()+2){
        	for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            	PointType pt2;
            	pt2.x = cloud_filterd->points[*pit].x, pt2.y = cloud_filterd->points[*pit].y, pt2.z = cloud_filterd->points[*pit].z;
            	pt2.intensity = (float)(j+1);
				cluster_cloud3.push_back(pt2);}
		}
        j++; 
    }
    double sum_x=0; 
    double sum_y=0;
    double sum_z=0;
	visualization_msgs::Marker mean_point;
	mean_point.header.frame_id = "map";
	mean_point.header.stamp = ros::Time::now();
	mean_point.ns = "points";
	mean_point.action = visualization_msgs::Marker::ADD;
	mean_point.type = visualization_msgs::Marker::POINTS;
	mean_point.pose.orientation.w = 1;
	mean_point.id = 0;
	mean_point.color.g = 1.0f; 
	mean_point.color.a = 1.0;
    mean_point.scale.x = 0.1;
    mean_point.scale.y = 0.1;
	
	vector<geometry_msgs::Point> mean_p;
    if (cluster_cloud2.size() != 0){
        for (size_t i=0; i<cluster_cloud2.size(); i++){
            cout << cluster_cloud2[i].x << endl;
            sum_x += cluster_cloud2[i].x;
            sum_y += cluster_cloud2[i].y;
            sum_z += cluster_cloud2[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud2.size();
        p_.y = sum_y / cluster_cloud2.size();
        p_.z = sum_z / cluster_cloud2.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
		sum_x = 0; sum_y = 0; sum_z = 0;
    }

	if (cluster_cloud1.size() != 0){
        for (size_t i=0; i<cluster_cloud1.size(); i++){
            cout << cluster_cloud1[i].x << endl;
            sum_x += cluster_cloud1[i].x;
            sum_y += cluster_cloud1[i].y;
            sum_z += cluster_cloud1[i].z;
        
        }
    
        geometry_msgs::Point p_;

        p_.x = sum_x / cluster_cloud1.size();
        p_.y = sum_y / cluster_cloud1.size();
        p_.z = sum_z / cluster_cloud1.size();

        mean_point.points.push_back(p_);
		mean_p.push_back(p_);
    }

    //point_pub_.publish(mean_point);
	return mean_p;


	// never happen
	Result_cloud += cluster_cloud1;
	Result_cloud += cluster_cloud2;
	Result_cloud += cluster_cloud3; 
	cout << mean_p.size() << endl;
	
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(Result_cloud, cloud_p);

    sensor_msgs::PointCloud2 result;
    pcl_conversions::fromPCL(cloud_p, result);
    result.header.frame_id = "map";
    //pub_.publish(result);
}

