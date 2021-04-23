#include <math.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// For global path planning
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <cstdlib>
#include <pluginlib/class_loader.hpp>

//service
#include "find_frontier/InitPos.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class findFrontier {
public:
	enum class Action {
		// Turn left, turn right, move forward enough for navigation
		LEFT,
		RIGHT,
		FWD
	};
	enum class Direction {
		EAST,
		SOUTH,
		WEST,
		NORTH
	};
	// all enum followed hiprlgrid convention
	findFrontier(tf2_ros::Buffer& tf);
	virtual ~findFrontier();
	void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
	void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
	bool PoseUpdate(find_frontier::InitPos::Request &req, find_frontier::InitPos::Response& res);
	bool InitPoseUpdate(find_frontier::InitPos::Request &req, find_frontier::InitPos::Response& res);
	void timerCallback(const ros::TimerEvent& event);
	void makeActionPlan(const std::vector<geometry_msgs::PoseStamped> plan, const nav_msgs::OccupancyGrid::ConstPtr& msg);
	//void makeAction(const std::vector<int> index_plan, std_msgs::Int32MultiArray& action_list);
	int findGridIndex(const geometry_msgs::PoseStamped position);
	void addGoLeftAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoRightAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoUpAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoDownAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	//void PoseSubCb(geometry_msgs::PoseStampedConstPtr pose);

private:
	//ros::Subscriber pose_sub;
	ros::Subscriber navigation_map_sub;
	ros::ServiceServer init_pos_service, pos_service;
	ros::Publisher pub, action_plan_pub;
	ros::Publisher pub_markers;
	ros::Timer timer;
	geometry_msgs::PoseStamped *pose_ptr_, *g_initial_pose_ptr;
	int IsPoseUpdated, counter, size_x, size_y, agent_dir;
	bool g_isInit;
	ros::Publisher marker_pub;
	ros::Publisher goal_pub;

	float goal_x;
	float goal_y;
	float goal_w;
	float map_size = 100;
	float map_scale = 3;
	geometry_msgs::PoseStamped goal_position_;

	boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
	std::vector<geometry_msgs::PoseStamped>* planner_plan_;
	costmap_2d::Costmap2DROS* planner_costmap_ros_;
	ros::Publisher g_plan_pub_;
	tf2_ros::Buffer& tf_;

};
