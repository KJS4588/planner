#include "planner/planner.h"
#include "clustering.cpp"

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane

float LEFT_L = 3.5;
float LEFT_R = 0.8;
float RIGHT_L = -0.5;
float RIGHT_R = -2;
float FRONT_MIN_OFFSET = 0.3;
float FRONT_MAX_OFFSET = 7.0;

const unsigned int ORDER = 3;
const double ACCEPTABLE_ERROR = 0.01;

void Planner::initSetup(){
    odom_sub_ = nh_.subscribe("/odom", 10, &Planner::odomCallback, this);
    point_sub_ = nh_.subscribe("/velodyne_points", 10, &Planner::pointCallback, this);    
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);

	global_path_ = loadGlobalPath();
}

void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs){
    odom_x = odom_msgs->pose.pose.position.x;
    odom_y = odom_msgs->pose.pose.position.y;
    odom_z = odom_msgs->pose.pose.position.z;

    pose_x = odom_msgs->pose.pose.orientation.x;
    pose_y = odom_msgs->pose.pose.orientation.y;
    pose_z = odom_msgs->pose.pose.orientation.z;
    pose_w = odom_msgs->pose.pose.orientation.w;
    
    tf::Quaternion q(pose_x, pose_y, pose_z, pose_w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	yaw_d = yaw*180/M_PI;
    
}

void Planner::pointCallback(const sensor_msgs::PointCloud2ConstPtr &msg){
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg,*cloud);
    pcl::fromROSMsg(*msg,*cloud_XYZIR);

    double* left_coef = get<0>(getLine(cloud_XYZIR));
    double* right_coef = get<1>(getLine(cloud_XYZIR));

    Cluster c;
    sensor_msgs::PointCloud2 objects = c.clusterCallback(msg);
    point_pub_.publish(objects);

    setPlan(left_coef, right_coef, objects);
}


double Planner::calcDistance(double * coef, pcl::PointXYZ object_point){
    return abs(coef[3]*pow(object_point.x, 3) + coef[2]*pow(object_point.x, 2) + coef[1]*pow(object_point.x, 1) + coef[0] - object_point.y);
}

void Planner::setPlan(double* left_coef, double* right_coef, sensor_msgs::PointCloud2 objects) {
	return;
	/*
    pcl::PointXYZ object_point = getClosestObject(objects);

    double dist_l = calcDistance(left_coef, object_point); 
    double dist_r = calcDistance(right_coef, object_point);

    if (dist_l >= dist_r) { // use the road between obstacle and left_lane
    
    } else { // dist_r > dist_l -> use the road between obstacle and right_}
		//pcl::PointXYZ Planner::getClosestObject(sensor_msgs::PointCloud2 objects)
	}
	*/
}

/*
pcl::PointXYZ Planner::getClosestObject(sensor_msgs::PointCloud2 objects){
	return 
}
*/


tuple<double*, double*> Planner::getLine(pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr &cloud_XYZIR) {
    vector<Point> result_points;
    //cout << cloud_XYZIR->points.size();
    
    for (std::size_t j = 0; j < cloud_XYZIR->points.size(); ++j){
        velodyne_pointcloud::PointXYZIR pt_point = cloud_XYZIR->points[j];
        if (MIN_INTEN <= pt_point.intensity && pt_point.intensity <= MAX_INTEN) {
            Point point(pt_point.x, pt_point.y, pt_point.z, pt_point.intensity, pt_point.ring); 
            result_points.push_back(point);
        }
    }

    vector<LanePoint> temp_left_lane;
    vector<LanePoint> temp_right_lane;

    for (auto point : result_points) {
		LanePoint lp(point.x, point.y, point.layer);

        if (LEFT_R < point.y && point.y < LEFT_L && FRONT_MIN_OFFSET < point.x && point.x < FRONT_MAX_OFFSET){
            temp_left_lane.push_back(lp);
        }
        else if (RIGHT_R < point.y && point.y < RIGHT_L && FRONT_MIN_OFFSET < point.x && point.x < FRONT_MAX_OFFSET){
            temp_right_lane.push_back(lp);
        }
    }
    
    // for sorting
    vector<vector<LanePoint>> left_layer_list(16);
    vector<vector<LanePoint>> right_layer_list(16);

    // index == layer
    for (auto point : temp_left_lane) {
        left_layer_list[point.layer].push_back(point);
    }

    for (auto point : temp_right_lane) {
        right_layer_list.at(point.layer).push_back(point);
    }
    
    //   +
    //   x     + y -
    //   -
    vector<LanePoint> left_lane; 
    vector<LanePoint> right_lane; 

    int left_invalidated = 0;

    // calculate mean point and push it into left_lane, right_lane
    for (auto lpv : left_layer_list) {
        LanePoint lp;

        if (lpv.size() != 0) {
            float sum_x = 0.0;
            float sum_y = 0.0;

            for (auto lpoint : lpv) {
                sum_x += lpoint.x;
                sum_y += lpoint.y;
            }

            lp.x = sum_x / lpv.size();
            lp.y = sum_y / lpv.size();
            left_lane.push_back(lp);
        } else {
            lp.x = -1000.0;
            lp.y = -1000.0;
            left_lane.push_back(lp);
            left_invalidated++;
        }
    }

    double left_coef[ORDER + 1];

    if (left_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[left_lane.size()];
        double yData[left_lane.size()];

        for (int i = 0 ; i < left_lane.size() ; i++) {
            xData[i] = left_lane.at(i).x;
            yData[i] = left_lane.at(i).y;
        }

        result = polyfit(xData, yData, left_lane.size(), ORDER, left_coef);

		/*
        cout << endl;
        cout << "left lane" << endl;
        cout << left_coef[3] << " : " << left_coef[2] << " : " << left_coef[1] << " : " << left_coef[0] << endl;
        cout << endl;
		*/


    } else { // less than 4 -> use previous poly
    }
    
    int right_invalidated = 0;

    // calculate mean point and push it into left_lane, right_lane
    for (auto lpv : right_layer_list) {
        LanePoint lp;

        if (lpv.size() != 0) {
            float sum_x = 0.0;
            float sum_y = 0.0;

            for (auto lpoint : lpv) {
                sum_x += lpoint.x;
                sum_y += lpoint.y;
            }

            lp.x = sum_x / lpv.size();
            lp.y = sum_y / lpv.size();
            right_lane.push_back(lp);
        } else {
            lp.x = -1000.0;
            lp.y = -1000.0;
            right_lane.push_back(lp);
            right_invalidated++;
        }
    }

    double right_coef[ORDER + 1];

    if (right_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[right_lane.size()];
        double yData[right_lane.size()];

        for (int i = 0 ; i < right_lane.size() ; i++) {
            xData[i] = right_lane.at(i).x;
            yData[i] = right_lane.at(i).y;
        }

        result = polyfit(xData, yData, right_lane.size(), ORDER, right_coef);


		/*
        cout << endl;
        cout << "right lane" << endl;
        cout << right_coef[3] << " : " << right_coef[2] << " : " << right_coef[1] << " : " << right_coef[0] << endl;
        cout << endl;
		*/



    } else { // less than 4 -> use previous poly
    }
    
    vector<geometry_msgs::Point> waypoint;
    float ld = 1.5;
    
    for (int i = 0; i < 10; i++){
        
        float waypoint_y_l = left_coef[3]*ld*ld*ld + left_coef[2]*ld*ld + left_coef[1] * ld + left_coef[0];
        float waypoint_y_r = right_coef[3]*ld*ld*ld + right_coef[2]*ld*ld + right_coef[1] * ld + right_coef[0];
        //cout << waypoint_y_l << "            " << waypoint_y_r << endl;
        //cout << ld << endl;
        ld += 0.3;
        geometry_msgs::Point p;
        p.x = ld;
        p.y = (waypoint_y_l + waypoint_y_r)/2;
        p.z = 0;
        waypoint.push_back(p);
    } 

    return make_tuple(left_coef, right_coef);

	//visualize(left_lane, right_lane, waypoint);
}

vector<OdomDouble> Planner::loadGlobalPath() {
	vector<OdomDouble> path;

	ifstream file;
	file.open(GLOBAL_PATH_FILE);

	if (file.is_open()) {

		string line;

		while (getline(file, line)) {

			istringstream ss(line);

			vector<string> odomString;
			string stringBuffer;
			while (getline(ss, stringBuffer, ',')) {
				odomString.push_back(stringBuffer);
			}

			OdomDouble odomDouble(stod(odomString.at(0)), stod(odomString.at(1)), stod(odomString.at(2)), stod(odomString.at(3)), stod(odomString.at(4)), stod(odomString.at(5)), stod(odomString.at(6)));
			path.push_back(odomDouble);
		}

		file.close();
	}

	printGlobalPath(path);

	return path;
}

void Planner::printGlobalPath(vector<OdomDouble> path) {
	cout << "global path loading..." << endl;

	for (auto o : path) {
		cout << endl;
		cout << "x : " << o.getX() << endl;
		cout << "y : " << o.getY() << endl;
		cout << "z : " << o.getZ() << endl;
		cout << "o_x : " << o.getOX() << endl;
		cout << "o_y : " << o.getOY() << endl;
		cout << "o_z : " << o.getOZ() << endl;
		cout << "o_w : " << o.getOW() << endl;
	}

	cout << "global path loaded successfully" << endl;
	cout << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    Planner pl;
    pl.initSetup();
    ros::spin();
}
