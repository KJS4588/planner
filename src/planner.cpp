#include "planner/planner.h"
#include "clustering.cpp"

#define MIN_INTEN 25	// minimum value of intensity to detect lane
#define MAX_INTEN 70	// maximum value of intensity to detect lane


void Planner::initSetup(){
    point_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_object", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

	aligned_sub_ = nh_.subscribe("/aligned_points", 10, &Planner::alignedCallback, this);

	loadGlobalPath();
}

void Planner::alignedCallback(const sensor_msgs::PointCloud2ConstPtr& aligned_points) {
	vector<geometry_msgs::Point> obs_points = Cluster().cluster(aligned_points);

	if (obs_points.size() > 0) {
		int closest_index_0 = getClosestPoint(obs_points.at(0));
		int closest_index_1 = getClosestPoint(obs_points.at(1));

		
	}

}

// set final path using global path and local path
void Planner::setPlan() {

	return;
}

//return index
int Planner::getClosestPoint(geometry_msgs::Point p) {
	
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

template <class T>
T Planner::getDist(T point_1, T point_2){
	return sqrt(pow(point_1.x - point_2.x) + pow(point_1.y - point_2.y));
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
