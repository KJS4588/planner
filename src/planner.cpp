#include "planner/planner.h"
#include "clustering.cpp"

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane


void Planner::initSetup(){
    obstacle_sub_ = nh_.subscribe("/velodyne_obstacles", 10, &Planner::obstacleCallback, this);

    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);

	global_path_ = loadGlobalPath();
}

// detect obstacles
void Planner::obstacleCallback(const pcl::PointCloud<VPoint>::ConstPtr& obstacles) {

	vector<VPoint> obs_points;

	for (auto point : *obstacles) {
		double o_x = point.x;
		double o_y = point.y;

		double dist = sqrt(o_x*o_x + o_y*o_y);

		if (dist < DIST_OBS_HP) {
			obs_points.push_back(point);
		}
	}

	vector<VPoint> obs_vector;
	vector<vector<VPoint>> result_points;

	cout << "############################" << endl;

	for (int i=0;i<obs_points.size();i++) {
		if (i != obs_points.size()-1) { // exclude the last point TODO:
			double cur_x = obs_points.at(i).x;
			double cur_y = obs_points.at(i).y;

			double next_x = obs_points.at(i+1).x;
			double next_y = obs_points.at(i+1).y;

			double dist = sqrt(pow(cur_x-next_x, 2) + pow(cur_y-next_y, 2));

			if (dist < CLUSTER_HP) {
				obs_vector.push_back(obs_points.at(i));
			} else {
				obs_vector.push_back(obs_points.at(i));
				result_points.push_back(obs_vector);
				obs_vector.clear();
			}
		}
	}

	cout << "result_points_size -> " << result_points.size() << endl;

	for (int i=0;i<result_points.size();i++) {

		for (auto point : result_points.at(i)) {
			cout << point << endl;
		}

		cout << "index -> " << i << endl;
		cout << endl;
	}

	cout << "############################" << endl;

	// makeLocalPath(result_points);
}

// make local path
void Planner::makeLocalPath(vector<vector<VPoint>> points) {

}

// set final path using global path and local path
void Planner::setPlan() {
	return;
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

			OdomDouble odomDouble(stod(odomString.at(0)), stod(odomString.at(1)), stod(odomString.at(2)));
			path.push_back(odomDouble);
		}

		file.close();
	}

	printGlobalPath(path);

	return path;
}

void Planner::printGlobalPath(vector<OdomDouble> path) {
	cout << "global path loaded successfully" << endl;
	cout << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    Planner pl;
    pl.initSetup();
    ros::spin();
}
