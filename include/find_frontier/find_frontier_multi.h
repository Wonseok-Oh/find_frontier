#include <math.h>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <map>
#include <algorithm>
#include <iterator>
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
#include "find_frontier/InitPosList.h"

// message
#include "find_frontier/ActionArray.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class findFrontierMulti {
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
	findFrontierMulti(tf2_ros::Buffer& tf, std::string process_num);
	virtual ~findFrontierMulti();
	void selectFrontierPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void clusterFrontierPoints();
	bool allocateGoalsByDistance();
	bool allocateGoalsByPlan();
	void processAndPublishGoals();
	void publishEmptyPlan();
	void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	int makePlan(const geometry_msgs::PoseArray& goal, std::vector<std::vector<geometry_msgs::PoseStamped> >& plan);
	void publishPlan(const std::vector<std::vector<geometry_msgs::PoseStamped> >& path);
	bool PoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res);
	bool InitPoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res);
	void timerCallback(const ros::TimerEvent& event);
	void makeActionPlan(const std::vector<std::vector<geometry_msgs::PoseStamped> > plan, const nav_msgs::OccupancyGrid::ConstPtr& msg);
	//void makeAction(const std::vector<int> index_plan, std_msgs::Int32MultiArray& action_list);
	int findGridIndex(const geometry_msgs::PoseStamped position);
	void addGoLeftAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoRightAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoUpAction(std_msgs::Int32MultiArray& action_plan, int *dir);
	void addGoDownAction(std_msgs::Int32MultiArray& action_plan, int *dir);

	/*
	 * getPlanAndSize:
	 * Generate plan to goal for agent[id] and return the generated plan's size.
	 * If fail to generate plan or non-sense goal is given, return -1
	 */
	int getPlanAndSize(const geometry_msgs::Pose& goal, std::vector<geometry_msgs::PoseStamped>& plan, int id);
	//void PoseSubCb(geometry_msgs::PoseStampedConstPtr pose);

private:
	//ros::Subscriber pose_sub;
	ros::NodeHandle nh;
	ros::Subscriber navigation_map_sub;
	ros::ServiceServer init_pos_service, pos_service;
	ros::Publisher pub;

	ros::Publisher pub_markers;
	ros::Timer timer;
	geometry_msgs::PoseArray *pose_ptr_, *g_initial_pose_ptr;
	//geometry_msgs::PoseStamped *pose_ptr_, *g_initial_pose_ptr;
	int IsPoseUpdated, counter, size_x, size_y;
	int num_agents;
	std::string m_process_num;
	int* agent_dir;
	bool g_isInit;
	ros::Publisher marker_pub;
	ros::Publisher goal_list_pub;

	std::vector<float> goal_x;
	std::vector<float> goal_y;
	float goal_w;

	float map_start_x, map_start_y;

	// frontier points index lists
	std::vector<int> FrontierList;

	// frontier points index with numbering (0: index, 1: order of the index)
	std::vector<std::vector<int> > FrontierList_temp;

	// element: a cluster's indices vector
	std::vector<std::vector<int> > cluster;

	// key for agent_id, vector for cluster
	std::map<int, std::vector<int> > destinationCluster;

	// visualization markers
	visualization_msgs::Marker points;
	visualization_msgs::Marker cluster_points;

	float map_size = 100;
	float map_scale = 3;
	geometry_msgs::PoseArray goal_positions_;

	boost::shared_ptr<nav_core::BaseGlobalPlanner>* planner_;
	pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
	std::vector<std::vector<geometry_msgs::PoseStamped> >* planner_plan_;
	costmap_2d::Costmap2DROS** planner_costmap_ros_;
	std::unique_ptr<ros::Publisher[]> g_plan_pub_list;
	ros::Publisher action_plan_list_pub;

	tf2_ros::Buffer& tf_;
	int max_cluster_size = 10;

};
