#include "find_frontier/global_planner.h"

namespace global_planner {
	GlobalPlanner::GlobalPlanner(tf2_ros::Buffer& tf):
			bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
			planner_plan_(NULL), latest_plan_(NULL), tf_(tf) {
		std::cout << "global_planner: init" << std::endl;
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
		std::string global_planner_;
		private_nh.param("base_global_planner", global_planner_, std::string("navfn/NavfnROS"));
		planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
		latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
		g_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
		goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&GlobalPlanner::goalCB, this, _1));
		agent_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("agent_pos", 1, boost::bind(&GlobalPlanner::agentPosCB, this, _1));
		planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
		planner_costmap_ros_->pause();
		try {
			planner_ = bgp_loader_.createInstance(global_planner_);
			planner_->initialize(bgp_loader_.getName(global_planner_), planner_costmap_ros_);
		} catch (const pluginlib::PluginlibException& ex) {
			ROS_FATAL("Failed to create %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner_.c_str(), ex.what());
			exit(1);
		}

		planner_costmap_ros_->start();
	}

	GlobalPlanner::~GlobalPlanner(){
		delete planner_costmap_ros_;
		delete planner_plan_;
		delete latest_plan_;
		planner_->~BaseGlobalPlanner();
	}

	void GlobalPlanner::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
		std::cout << "global_planner: goalCB entered" << std::endl;
		goal_.header.stamp = ros::Time::now();
		goal_.header.frame_id = "map";
		goal_.pose.position.x = goal->pose.position.x;
		goal_.pose.position.y = goal->pose.position.y;
		goal_.pose.position.z = goal->pose.position.z;
		goal_.pose.orientation.x = goal->pose.orientation.x;
		goal_.pose.orientation.y = goal->pose.orientation.y;
		goal_.pose.orientation.z = goal->pose.orientation.z;
		goal_.pose.orientation.w = goal->pose.orientation.w;
		makePlan(goal_, *planner_plan_);
	}


	void GlobalPlanner::agentPosCB(const geometry_msgs::PoseStamped::ConstPtr& agent_pos){
		agent_pos_.header.stamp = ros::Time::now();
		agent_pos_.header.frame_id = "map";
		agent_pos_.pose.position.x = agent_pos->pose.position.x;
		agent_pos_.pose.position.y = agent_pos->pose.position.y;
		agent_pos_.pose.position.z = agent_pos->pose.position.z;
		agent_pos_.pose.orientation.x = agent_pos->pose.orientation.x;
		agent_pos_.pose.orientation.y = agent_pos->pose.orientation.y;
		agent_pos_.pose.orientation.z = agent_pos->pose.orientation.z;
		agent_pos_.pose.orientation.w = agent_pos->pose.orientation.w;
	}

	bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
		plan.clear();
		if (planner_costmap_ros_ == NULL){
			ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
			return false;
		}

		// get the starting pose of the robot
		std::vector<geometry_msgs::PoseStamped> global_plan;
		if (!planner_->makePlan(agent_pos_, goal, plan) || plan.empty()){
			ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
			return false;
		}
		publishPlan(plan);
		return true;
	}

	void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
	    nav_msgs::Path gui_path;
	    gui_path.poses.resize(path.size());

	    if (path.empty()){
	    	gui_path.header.frame_id = "map";
	    	gui_path.header.stamp = ros::Time::now();
	    } else {
	    	gui_path.header.frame_id = path[0].header.frame_id;
	    	gui_path.header.stamp = path[0].header.stamp;
	    }

	    for (unsigned int i = 0; i < path.size(); i++){
	    	gui_path.poses[i] = path[i];
	    }

	    g_plan_pub_.publish(gui_path);
	}
}
