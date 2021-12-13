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
#include <utility>
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
#include "find_frontier/PlanningID.h"

// message
#include "find_frontier/ActionArray.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class findFrontierMulti {
public:

	class Vertex {
	public:
		struct Compare {
				bool operator() (const std::pair<Vertex*, int> &a, const std::pair<Vertex*, int> &b){
					return a.second < b.second;
				}
		};

		struct isTaskId {
			isTaskId(const Vertex& a_vertex){
				type_id.assign(a_vertex.m_type_id);
			}
			std::string type_id;

			bool operator()(const Vertex& vertex){
				return vertex.m_type_id.compare(type_id) == 0;
			}
		};

		Vertex(){}
		Vertex(std::string type_id){
			m_type_id = type_id;
			m_x = 0;
			m_y = 0;
			m_parent = NULL;
		}
		// type_id: e.g., r3 for robot with id 3, t2 for task with id 2
		Vertex(std::string type_id, double x, double y){
			m_type_id = type_id; // robot or task
			m_x = x;
			m_y = y;
			m_parent = NULL;
		}
		void addEdge(Vertex* dst, int weight){
			std::pair<Vertex*, int> cost_element;
			//cost_element.first = new Vertex;
			cost_element.first = dst;
			cost_element.second = weight;
			m_cost.insert(cost_element);
			if (m_type_id.front() == dst->m_type_id.front()){
				//std::cout << "same type: add edges each other" << std::endl;
				std::pair<Vertex*, int> cost_element2;
				//cost_element2.first = new Vertex;
				cost_element2.first = this;
				cost_element2.second = weight;

				dst->m_cost.insert(cost_element2);
				//std::cout << "dst " << dst->m_type_id << " address: " << dst << std::endl;
				//std::cout << "dst to this connected: " << dst->isConnected(this) << std::endl;
				//dst->printEdge();
			}
			return;
		}

		void printEdge(){
			std::cout << "connected edges to " << m_type_id << ": " << std::endl;
			std::set<std::pair<Vertex*, int> >::iterator iter;
			iter = m_cost.begin();
			for(;iter != m_cost.end(); iter++){
				//std::cout << "address: " << iter->first << std::endl;
				std::cout << "   " << iter->first->m_type_id << " with cost " << iter->second << std::endl;
			}
		}

		bool isConnected(Vertex* dst){
			return (this->m_cost.end() != std::find_if(m_cost.begin(), m_cost.end(),
					[dst](const std::pair<Vertex*, int>& p){
				return p.first->m_type_id.compare(dst->m_type_id) == 0;
			}));
		}
		/*
		 * removeEdge:
		 * remove edge from this --> dst essentially
		 * if there is no such edge, just return with no edge to remove message.
		 * if ther exists and dst--> this is also connected, remove dst-->this too.
		 */
		void removeEdge(Vertex* dst){
			std::set<std::pair<Vertex*, int> >::iterator iter_src, iter_dst;

			// iter_src: iterator pointing this->m_cost's element which first element is dst
			iter_src = std::find_if(m_cost.begin(), m_cost.end(),
					[dst](const std::pair<Vertex*, int>& p){
				return p.first->m_type_id.compare(dst->m_type_id) == 0;
			});

			// in case this vertex is not connected to dst
			if (iter_src == m_cost.end()){
				std::cout << m_type_id << " is not connected to " << dst->m_type_id << std::endl;
				return;
			}

			if (m_type_id.front() == 't'){
				// iter_dst: iterator pointing dst->m_cost's element which first element is "this"
				int cost = iter_src->second;
				iter_dst = dst->m_cost.find(std::pair<Vertex*, int>(this, cost));

				// in case iter_dst is found
				// == in case the edge from dst -> src is connected
				if (iter_dst != dst->m_cost.end()){
					// remove edge from dst --> src
					//delete iter_dst->first;
					dst->m_cost.erase(iter_dst);
				}
			}
			// remove edge from this --> dst
			//delete iter_src->first;
			m_cost.erase(iter_src);
			return;
		}
		double m_x, m_y;
		std::string m_type_id;
		std::vector<Vertex*> m_child;
		std::set< std::pair<Vertex*, int>, Compare> m_cost;
		Vertex* m_parent;
	};

	enum class Action {
		// Turn left, turn right, move forward for navigation, open for plan
		LEFT,
		RIGHT,
		FWD,
		PICKUP,
		DROP,
		OPEN
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
	int makePlanBeforeBox();

	void publishPlan(const std::vector<std::vector<geometry_msgs::PoseStamped> >& path);
	bool PoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res);
	bool InitPoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res);
	bool allocateTask(find_frontier::PlanningID::Request &req, find_frontier::PlanningID::Response& res);
	void timerCallback(const ros::TimerEvent& event);

	// used in exploration
	void makeActionPlan(const std::vector<std::vector<geometry_msgs::PoseStamped> > plan,
			const nav_msgs::OccupancyGrid::ConstPtr& msg);

	// used in planning
	void makeActionPlan(const std::vector<geometry_msgs::PoseStamped> plan, std_msgs::Int32MultiArray& index_plan,
			std_msgs::Int32MultiArray& action_plan, int id);
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
	int getPlanAndSize(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start, std::vector<geometry_msgs::PoseStamped>& plan);

	// return minimum cost in the branch
	int getMinCostInGraph(Vertex* vertex, Vertex** src, Vertex** dst);

	// construct graph
	void generateGraph(find_frontier::PlanningID::Request &req);

	// remove a task specified by task_id from the graph
	void removeTaskFromGraph(std::string task_id);

	// print generated all MSTs
	void printMST();

	// print MST starting from node in preorder
	void printMST(Vertex* node);

	void planningMapCb();

	// generate plan from MST
	void generatePlanFromMST(find_frontier::PlanningID::Response &res);
	// generate plan from MST starting from node in preorder
	void generatePlanFromMST(Vertex* node, std::vector<geometry_msgs::PoseStamped>& plan,
			std_msgs::Int32MultiArray& index_plan, std_msgs::Int32MultiArray& action_plan, int id);

	int makePlanBeforeBox(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
			std::vector<geometry_msgs::PoseStamped>& plan, std_msgs::Int32MultiArray& index_plan,
			std_msgs::Int32MultiArray& action_plan, int id);

	//void PoseSubCb(geometry_msgs::PoseStampedConstPtr pose);

private:
	//ros::Subscriber pose_sub;
	ros::NodeHandle nh;
	ros::Subscriber navigation_map_sub, m_planning_map_sub;
	ros::ServiceServer init_pos_service, pos_service, m_allocate_task_service;
	ros::Publisher pub;

	ros::Publisher pub_markers;
	ros::Timer timer;
	geometry_msgs::PoseArray *pose_ptr_, *g_initial_pose_ptr;
	//geometry_msgs::PoseStamped *pose_ptr_, *g_initial_pose_ptr;
	int IsPoseUpdated, counter, size_x, size_y;
	int num_agents; // number of agents selected explore
	std::string m_process_num;
	std::vector<int> agent_dir;
	bool g_isInit;
	ros::Publisher marker_pub;
	ros::Publisher goal_list_pub;

	std::vector<float> goal_x;
	std::vector<float> goal_y;
	float goal_w;

	float map_start_x, map_start_y;

	// frontier points index lists
	std::vector<int> FrontierList, explore_agent_id_list, m_min_cost, plan_agent_id_list;

	// frontier points index with numbering (0: index, 1: order of the index)
	std::vector<std::vector<int> > FrontierList_temp;

	// element: a cluster's indices vector
	std::vector<std::vector<int> > cluster;

	// key for agent_id, vector for cluster
	std::map<int, std::vector<int> > destinationCluster;

	// visualization markers
	visualization_msgs::Marker points;
	visualization_msgs::Marker cluster_points;

	// Task vertices
	std::vector<Vertex> m_tasks;

	// Minimum Spanning Tree(MST) root for task allocation
	std::vector<Vertex> m_root;

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
	int total_num_agents = 3;
	int m_num_tasks, m_num_planning_agent;
	int m_planning_mode;
	bool m_is_planning;

};
