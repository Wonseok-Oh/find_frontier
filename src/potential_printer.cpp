#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
void potential_map_callback0(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int idx;
	std::cout << "map0 received" << std::endl;
	for (int j = msg->info.height - 1; j >= 0; j--){
		for (int i = 0; i < msg->info.width; i++){
			idx = i + msg->info.width * j;
			std::cout << static_cast<int>(msg->data[idx]) << " ";
			if (i == msg->info.width -1){
				std::cout << std::endl;
			}
		}
	}
}
void potential_map_callback1(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int idx;
	std::cout << "map1 received" << std::endl;
	for (int j = msg->info.height - 1; j >= 0; j--){
		for (int i = 0; i < msg->info.width; i++){
			idx = i + msg->info.width * j;
			std::cout << static_cast<int>(msg->data[idx]) << " ";
			if (i == msg->info.width -1){
				std::cout << std::endl;
			}
		}
	}
}
void potential_map_callback2(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	int idx;
	std::cout << "map2 received" << std::endl;
	for (int j = msg->info.height - 1; j >= 0; j--){
		for (int i = 0; i < msg->info.width; i++){
			idx = i + msg->info.width * j;
			std::cout << static_cast<int>(msg->data[idx]) << " ";
			if (i == msg->info.width -1){
				std::cout << std::endl;
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "potential_printer");
	ros::NodeHandle nh;
	ros::Subscriber sub0 = nh.subscribe("/hiprl_explore0/GlobalPlanner0/potential", 1, potential_map_callback0);
	ros::Subscriber sub1 = nh.subscribe("/hiprl_explore0/GlobalPlanner1/potential", 1, potential_map_callback1);
	ros::Subscriber sub2 = nh.subscribe("/hiprl_explore0/GlobalPlanner2/potential", 1, potential_map_callback2);

	ros::spin();
	return 0;
}
