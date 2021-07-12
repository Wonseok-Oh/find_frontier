#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>

ros::Publisher g_image_pub;

#define BOX 110
#define BALL -2
#define KEY -120
#define FLOOR 0
#define UNKNOWN -1
#define WALL 100
#define GOAL 50

bool objectMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map){
	cv::Mat image;
	if (map->info.width > 0 && map->info.height > 0){
		image = cv::Mat::zeros(map->info.width, map->info.height, CV_8UC3);
	} else {
		std::cerr << "Invalid gridmap size" << std::endl;
		return false;
	}

	for (int i = 0; i < map->info.width*map->info.height; i++){
		int x = i % map->info.width;
		int y = i / map->info.width;
		switch(map->data[i]){
		// all data is from below link:
		// https://m.blog.naver.com/PostView.nhn?blogId=leehoratius&logNo=220232390526&proxyReferer=https:%2F%2Fwww.google.com%2F
		case BOX: //lawn green
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 252;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 124;
			break;

		case BALL: // yellow
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 255;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 255;
			break;

		case KEY: // red
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 255;
			break;

		case FLOOR: // white
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 255;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 255;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 255;
			break;

		case UNKNOWN: // sea green
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 87;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 139;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 46;
			break;

		case WALL: // black
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 0;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 0;
			break;

		case GOAL: // pink
			image.at<cv::Vec<unsigned char, 3>>(x,y)[0] = 203;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[1] = 192;
			image.at<cv::Vec<unsigned char, 3>>(x,y)[2] = 255;
			break;

		default:
			std::cout << "objmap_to_image_converter: this should not happen in switch" << std::endl;
			break;
		}
	}
	rotate(image, image, 2);
	cv_bridge::CvImage out_msg;
	out_msg.header.frame_id = "map";
	out_msg.header.stamp = ros::Time::now();
	out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	out_msg.image = image;
	g_image_pub.publish(out_msg);

	return true;

}

int main(int argc, char **argv){
	ros::init(argc, argv, "object_map_converter");
	std::string process_num = std::string(argv[1]);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("object_map" + process_num, 1, objectMapCallback);
	g_image_pub = n.advertise<sensor_msgs::Image>("object_img_map" + process_num, 1);
	ros::spin();
	return 0;
}
