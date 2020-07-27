#include "planner/planner.h"
#include "clustering.cpp"


void Planner::initSetup(){
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);
	aligned_sub_ = nh_.subscribe("/aligned_points", 10, &Planner::alignedCallback, this);
	odom_sub_ = nh_.subscribe("/odom", 10, &Planner::odomCallback, this);
	marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/obj_points", 10);
	loadGlobalPath();
}

// get current pose
void Planner::odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg){
	lx = odomsg->pose.pose.position.x;
	ly = odomsg->pose.pose.position.y;
	lz = odomsg->pose.pose.position.z;
}

void Planner::alignedCallback(const sensor_msgs::PointCloud2ConstPtr& aligned_points) {
	vector<geometry_msgs::Point> obs_points = Cluster().cluster(aligned_points, -5, 3, -4.0, 1);
	visualize(obs_points);

	if (obs_points.size() >= 2) { // two obstacles detected -> set new global path
		if (obs_detect_flag_ < 1) { // execute below only once

			double m = getLinearValues().at(0);
			double n = getLinearValues().at(1);
			
			// closer symmetric point
			double x_1 = obs_points.at(0).x - 2*m*(m*obs_points.at(0).x - obs_points.at(0).y + n) / (m*m + 1);
			double y_1 = obs_points.at(0).y + 2*(m*obs_points.at(0).x - obs_points.at(0).y + n) / (m*m + 1);

			// farther symmetric point
			double x_2 = obs_points.at(1).x - 2*m*(m*obs_points.at(1).x - obs_points.at(1).y + n) / (m*m + 1);
			double y_2 = obs_points.at(1).y + 2*(m*obs_points.at(1).x - obs_points.at(1).y + n) / (m*m + 1);
		
			// mean point of symmetric points
			double mean_x = (x_1 + x_2) / 2;
			double mean_y = (y_1 + y_2) / 2;

			// closest index of global path from first obstacle
			int closest_obs_index = -1;

			geometry_msgs::Point local_point;
			local_point.x = lx;
			local_point.y = ly;

			if (getDist(local_point, obs_points.at(0)) > getDist(local_point, obs_points.at(1))) {
					closest_obs_index = 1;
			} else {
					closest_obs_index = 0;
			}


			int closest_index = getClosestPointIndex(obs_points.at(closest_obs_index));
			cout << "Closest_index : " << closest_index << endl;

			// keep the last point of global_path
			OdomDouble g_last = global_path_.back();

			// erase points in global_path
			int count = 0;
			int origin_size_global_path = global_path_.size();

			while(count < origin_size_global_path-closest_index+4) {
				global_path_.pop_back();
				count++;
			}

			cout << "resized 1 global path -> " << global_path_.size() << endl;

			// add new path to global_path
			if (closest_obs_index == 0) {
				global_path_.push_back(OdomDouble(x_1, y_1));
				global_path_.push_back(OdomDouble(mean_x, mean_y));
				global_path_.push_back(OdomDouble(x_2, y_2));
			}else {
				global_path_.push_back(OdomDouble(x_2, y_2));
				global_path_.push_back(OdomDouble(mean_x, mean_y));
				global_path_.push_back(OdomDouble(x_1, y_1));
			}

			global_path_.push_back(g_last);

			cout << "resized 2 global path -> " << global_path_.size() << endl;

			savePath();

			nh_.setParam("/isGlobalPathChanged", true);
			obs_detect_flag_++;
		}
	}
}

// return a and b of linear
vector<double> Planner::getLinearValues() {
	double m = (global_path_.back().getY()-global_path_.front().getY()) / (global_path_.back().getX()-global_path_.front().getX());
	double n = -global_path_.front().getX()*m + global_path_.front().getY();

	vector<double> result;
	result.push_back(m);
	result.push_back(n);

	return result;
}

//return index
int Planner::getClosestPointIndex(geometry_msgs::Point p) {
	
	int result_index = -1; 
	double distance = 100.0;

	for(int i=0;i<global_path_.size();i++) {
		if(distance > getDist(p, global_path_.at(i))) {
			distance = getDist(p, global_path_.at(i));
			result_index = i;
		}
	}

	return result_index;
}

double Planner::getDist(geometry_msgs::Point point_1, OdomDouble point_2){
	return sqrt(pow(point_1.x - point_2.getX() , 2) + pow(point_1.y - point_2.getY() , 2));
}

double Planner::getDist(geometry_msgs::Point point_1, geometry_msgs::Point point_2){
	return sqrt(pow(point_1.x - point_2.x , 2) + pow(point_1.y - point_2.y , 2));
}

void Planner::loadGlobalPath() {
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

			OdomDouble odomDouble(stod(odomString.at(0)), stod(odomString.at(1)), stod(odomString.at(2)));
			global_path_.push_back(odomDouble);
		}

		file.close();
		cout << endl;
		cout << "load success" << endl;;
		cout << global_path_.size() << endl;
		cout << endl;
	}
}

void Planner::savePath() {
	if (access(NEW_GLOBAL_PATH_FILE, 0) == 0) {
		int result = remove(NEW_GLOBAL_PATH_FILE);
	}

    cout << "Obstacles detected" << endl;
    cout << "start saving new path..." << endl;
    
    ofstream file(NEW_GLOBAL_PATH_FILE);

    for (auto odom : this->global_path_) {
        double x = odom.getX();
        double y = odom.getY();
        double z = odom.getZ();

        if (file.is_open()) {
            string row = to_string(x) + "," + to_string(y) + "," + to_string(z) + "\n";
            file << row;
        }
    }

    file.close();

	cout << endl;
    cout << "--- saved new path successfully ---" << endl;
    cout << "text file path -> " << NEW_GLOBAL_PATH_FILE << endl;
    cout << "path size -> " << to_string(this->global_path_.size()) << endl;
    cout << "-------------------------------" << endl;
	cout << endl;
}

void Planner::visualize(vector<geometry_msgs::Point> obs_points){

	visualization_msgs::Marker points;

	points.header.frame_id = "map";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1; 
	points.scale.y = 0.1;
	points.color.a = 1.0;
	points.color.b = 1.0f;

	geometry_msgs::Point p;

	for (auto point : obs_points) {
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		points.points.push_back(p);
	}

	marker_pub_.publish(points);
}



int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    Planner pl;
    pl.initSetup();
    ros::spin();
}
