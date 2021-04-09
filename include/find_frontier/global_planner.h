#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <vector>
#include <string>
#include <ros/ros.h>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include <pluginlib/class_loader.hpp>

namespace global_planner {
	class GlobalPlanner {
	public:
		GlobalPlanner(tf2_ros::Buffer& tf);
		virtual ~GlobalPlanner();
		bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
		void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
		void agentPosCB(const geometry_msgs::PoseStamped::ConstPtr& agent_pos);
		void planThread();
		void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

	private:
		boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
		geometry_msgs::PoseStamped global_pose_;
		pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
		std::vector<geometry_msgs::PoseStamped>* planner_plan_;
		std::vector<geometry_msgs::PoseStamped>* latest_plan_;
		geometry_msgs::PoseStamped agent_pos_, goal_;
		ros::Subscriber goal_sub_, agent_pos_sub_;
		costmap_2d::Costmap2DROS* planner_costmap_ros_;
		tf2_ros::Buffer& tf_;
		ros::Publisher g_plan_pub_;

	};

}

#endif
