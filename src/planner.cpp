#include "planner/planner.h"
#include "clustering.cpp"

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane


void Planner::initSetup(){
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	loadGlobalPath();
}

// make local path
void Planner::makeLocalPath(vector<vector<VPoint>> points) {

}

// set final path using global path and local path
void Planner::setPlan() {
	return;
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
	}
}

void Planner::visualize(vector<OdomDouble> global_path){
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
	points.color.r = 1.0f;

	geometry_msgs::Point p;

	for (auto point : global_path) {
		p.x = point.getX();
		p.y = point.getY();
		p.z = point.getZ();
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
