#include "find_frontier/find_frontier_multi.h"

using namespace std;

findFrontierMulti::findFrontierMulti(tf2_ros::Buffer& tf, string process_num): tf_(tf), m_process_num(process_num),
		goal_x(0), goal_y(0), goal_w(1), IsPoseUpdated(0), counter(0), g_isInit(false), planner_(nullptr),
		bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), planner_costmap_ros_(NULL), size_x(0), size_y(0),
		num_agents(0), nh(ros::NodeHandle()), map_start_x(0), map_start_y(0),
		m_num_tasks(0), m_num_planning_agent(0), m_is_planning(true){
		navigation_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("navigation_map" + process_num, 1, &findFrontierMulti::mapConvert, this);
		//m_planning_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("planning_map" + process_num, 1, &findFrontierMulti::planningMapCb, this);

		init_pos_service = nh.advertiseService("init_pose_list_update" + process_num, &findFrontierMulti::InitPoseListUpdate, this);
		pos_service = nh.advertiseService("pose_list_update" + process_num, &findFrontierMulti::PoseListUpdate, this);
		m_allocate_task_service = nh.advertiseService("task_allocate" + process_num, &findFrontierMulti::allocateTask, this);
		//init_pos_sub = nh.subscribe("initial_pose", 1, &findFrontier::InitPoseUpdate, this);
		//sub = nh.subscribe<nav_msgs::OccupancyGrid>("spatial_map", 1, boost::bind(&findFrontier::mapConvert, this, _1) );
		timer = nh.createTimer(ros::Duration(0.2), &findFrontierMulti::timerCallback, this);
		marker_pub = nh.advertise<visualization_msgs::Marker>("FrontierLocationMarker" + process_num,1);
		goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("goal" + process_num,1, true);
		//g_plan_pub_list = nh.advertise<nav_msgs::Path>("global_plan" + process_num, 1);
		//action_plan_pub_list = nh.advertise<std_msgs::Int32MultiArray>("action_plan" + process_num, 1);
		action_plan_list_pub = nh.advertise<find_frontier::ActionArray>("action_plan" + process_num, 1);

	    planner_plan_ = new std::vector<std::vector<geometry_msgs::PoseStamped> >();
	    pose_ptr_ = new geometry_msgs::PoseArray;
	    g_initial_pose_ptr = new geometry_msgs::PoseArray;
	    agent_dir = vector<int>(total_num_agents);
	    planner_costmap_ros_ = new costmap_2d::Costmap2DROS*[total_num_agents];
		planner_ = new boost::shared_ptr<nav_core::BaseGlobalPlanner>[total_num_agents];
		m_planning_mode = find_frontier::PlanningID::Response::SEARCH;
}

void findFrontierMulti::planningMapCb(){

}
findFrontierMulti::~findFrontierMulti(){
	delete planner_plan_;
	delete pose_ptr_, g_initial_pose_ptr;
	for (int i = 0; i < num_agents; i++){
		delete planner_costmap_ros_[i];
	}
	delete planner_costmap_ros_;
	delete planner_;
	for (int i = 0; i < m_num_planning_agent; i++){
		set<pair<Vertex*, int> >::iterator iter = m_root[i].m_cost.begin();
		for (;iter != m_root[i].m_cost.end(); iter++){
			delete iter->first;
		}
	}

	for (int i = 0; i < m_tasks.size(); i++){
		set<pair<Vertex*, int> >::iterator iter = m_tasks[i].m_cost.begin();
		for (;iter != m_tasks[i].m_cost.end(); iter++){
			delete iter->first;
		}
	}


	//planner_->~BaseGlobalPlanner();
	//planner_.reset();
}
void findFrontierMulti::selectFrontierPoints(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	  goal_x.clear();
	  goal_y.clear();
	  goal_positions_.poses.clear();
	  cluster.clear();
	  FrontierList.clear();
	  FrontierList_temp.clear();
	  points.points.clear();
	  cluster_points.points.clear();


	  //cout << "mapConvert is called" << endl;
	  points = visualization_msgs::Marker();
	  points.header.frame_id ="/map";
	  points.header.stamp= ros::Time::now();
	  points.ns="frontier_points";
	  points.action = visualization_msgs::Marker::ADD;
	  points.pose.orientation.w=1.0;
	  points.id = 0;
	  points.type = visualization_msgs::Marker::POINTS;
	  points.scale.x = map_scale;
	  points.scale.y = map_scale;

	  cluster_points = visualization_msgs::Marker();
	  cluster_points.header.frame_id ="/map";
	  cluster_points.header.stamp= ros::Time::now();
	  cluster_points.ns="frontier_cluster_points";
	  cluster_points.action = visualization_msgs::Marker::ADD;
	  cluster_points.pose.orientation.w=1.0;
	  cluster_points.id = 0;
	  cluster_points.type = visualization_msgs::Marker::POINTS;
	  cluster_points.scale.x = map_scale;
	  cluster_points.scale.y = map_scale;

	  if (g_isInit == false && IsPoseUpdated == 0) return;
	  else{
	      //std::cout << "isInit is true" << std::endl;
		  map_start_x = g_initial_pose_ptr->poses[0].position.x;
		  map_start_y = g_initial_pose_ptr->poses[0].position.y;
	  }
	  size_x = msg->info.width; // # of grid
	  size_y = msg->info.height;
	  FrontierList.clear();
	  //Find the frontiers
	  for(int i=0;i<size_x*size_y;i++){
		  int x_ic = i%size_x;
		  int y_ic = i/size_x;

		  float x_c = float(x_ic+0.5)*map_scale-map_start_x;
		  float y_c = float(y_ic+0.5)*map_scale-map_start_y;
		  float distance = 2000;
		  if(IsPoseUpdated==1){
			  //std::cout << "Pose updated" << std::endl;
			  double cur_pose_x = pose_ptr_->poses[0].position.x - map_start_x;
			  double cur_pose_y = pose_ptr_->poses[0].position.y - map_start_y;
			  distance = sqrt(pow(cur_pose_x-x_c,2)+pow(cur_pose_y-y_c,2));
		  }
		  if(msg->data[i]==0){
			  if (x_ic >= 1 && x_ic <= size_x-2){
				  if (y_ic >=1 && y_ic <= size_x-2){
					  int nearones_5by5[25];
					  int nearones_3by3[9];
					  int count_occupied = 0;

					  int IsFrontier = 0;
					  for(int j=0;j<9;j++){
						  nearones_3by3[j]=i+size_x*(j/3-1)+(j%3-1);
					  }

					  for(int j=0;j<9;j++){
						  if(msg->data[nearones_3by3[j]]==-1){
							  IsFrontier = 1;
							  break;
						  }
						  //Mark as a frontier if unexplored space is near
					  }
					  if(x_ic >= 2 && x_ic <= size_x-3){
						  if(y_ic >= 2 && y_ic <= size_x-3){
							  for(int j=0;j<25;j++){
								  nearones_5by5[j]=i+size_x*(j/5-2)+(j%5-2);
							  }

							  if(IsFrontier==1){
								  for(int j=0;j<25;j++){
									  if(msg->data[nearones_5by5[j]]==100){
										  count_occupied++;
									  }
									  //Unmark if occupied space is near
								  }
								  //cout << "index: " << i << ", count_occupied: " << count_occupied << endl;
							  }
						  }
					  }
					  if(IsFrontier==1 && count_occupied <= 20){
						  FrontierList.push_back(i);
						  //std::cout << "frontier index i: " << i << std::endl;
					  }
				  }
			  }
		  }
	  }

	  g_plan_pub_list = unique_ptr<ros::Publisher[]>(new ros::Publisher[total_num_agents]);
	  // declare plan & action_plan publisher lists here
	  for (int i = 0; i < total_num_agents; i++){
		  g_plan_pub_list[i] = nh.advertise<nav_msgs::Path>(i + "global_plan" + m_process_num, 1);
	  }
	  IsPoseUpdated = 0;
	  return;
}

void findFrontierMulti::clusterFrontierPoints(){
	//conduct clustering
	ROS_INFO("Frontier Detected!");
	ROS_INFO("Frontier Size : %d", int(FrontierList.size()));
	//Frontier Detection is Done
	FrontierList_temp.resize(2);
	for(int i=0;i<FrontierList.size();i++){
		FrontierList_temp[0].push_back(FrontierList[i]);
		FrontierList_temp[1].push_back(i);
	}
	if(FrontierList.size()>1){
		vector<int> individualCluster;
		vector<int> node;
		node.push_back(0);
		individualCluster.push_back(0);
		while(1){
			vector<int> next_node;
			for(int i=0;i<node.size();i++){
				int baseNode_x = FrontierList[node[i]]%size_x;
				int baseNode_y = FrontierList[node[i]]/size_x;

				for(int j=0;j<FrontierList_temp[0].size();){
					int target_x = FrontierList_temp[0][j]%size_x;
					int target_y = FrontierList_temp[0][j]/size_x;


					int distance = pow(target_x-baseNode_x,2)+pow(target_y-baseNode_y,2);
					if (distance == 0){
						individualCluster.push_back(FrontierList_temp[1][j]);
					}

					if(distance<=2 && distance!=0){
						individualCluster.push_back(FrontierList_temp[1][j]);
						next_node.push_back(FrontierList_temp[1][j]);
						FrontierList_temp[0].erase(FrontierList_temp[0].begin()+j);
						FrontierList_temp[1].erase(FrontierList_temp[1].begin()+j);
					} else {
						j++;
					}
				}
			}
			node.clear();
			node = next_node;
			next_node.clear();

			// if you checked all of the frontier points for clustering
			if(FrontierList_temp[0].size()==0){
				  if (individualCluster.size() > 0){
					  // chunk cluster if the size is larger than max_cluser_size (10)
					  if (individualCluster.size() > max_cluster_size){
						  vector< vector<int> > cut_cluster;
						  vector<int> cut_cluster_seg;
						  int k = 0;
						  for (k = 0; (k+1)*max_cluster_size < individualCluster.size() ; k++){
							  cut_cluster_seg.clear();
							  cut_cluster_seg.assign(individualCluster.begin()+k*max_cluster_size, individualCluster.begin()+(k+1)*max_cluster_size);
							  cut_cluster.push_back(cut_cluster_seg);
							  cluster.push_back(cut_cluster_seg);
						  }
						  cut_cluster_seg.clear();
						  cut_cluster_seg.assign(individualCluster.begin()+k*max_cluster_size, individualCluster.end());
						  cut_cluster.push_back(cut_cluster_seg);
						  cluster.push_back(cut_cluster_seg);
					  } else {
						  cluster.push_back(individualCluster);
						  //cout << "individualCluster's size 1st: " << individualCluster.size() << endl;
					  }
				  }
				  individualCluster.clear();
				  break;
			} else {
				if(node.size()==0){
					if (individualCluster.size() > 0){
						// chunk cluster if the size is larger than max_cluser_size (10)
						if (individualCluster.size() > max_cluster_size){
							vector< vector<int> > cut_cluster;
							vector<int> cut_cluster_seg;
							int k = 0;
							for (k = 0; (k+1)*max_cluster_size < individualCluster.size() ; k++){
								cut_cluster_seg.clear();
								cut_cluster_seg.assign(individualCluster.begin()+k*max_cluster_size, individualCluster.begin()+(k+1)*max_cluster_size);
								cut_cluster.push_back(cut_cluster_seg);
								cluster.push_back(cut_cluster_seg);
							}
							cut_cluster_seg.clear();
							cut_cluster_seg.assign(individualCluster.begin()+k*max_cluster_size, individualCluster.end());
							cut_cluster.push_back(cut_cluster_seg);
							cluster.push_back(cut_cluster_seg);
						} else {
							cluster.push_back(individualCluster);
							//cout << "individualCluster's size 2nd: " << individualCluster.size() << endl;
						}
					}

					individualCluster.clear();
					node.push_back(FrontierList_temp[1][0]);
					FrontierList_temp[0].erase(FrontierList_temp[0].begin());
					FrontierList_temp[1].erase(FrontierList_temp[1].begin());
				}
			}
		}
	} else if (static_cast<int>(FrontierList.size()) == 1){
		vector<int> individualCluster;
		individualCluster.push_back(0);
		cluster.push_back(individualCluster);
	}
	//cout << "clustering complete, # of clusters: " << cluster.size() << endl;
	//Find the biggest cluster
	vector<int> maxClusterIdx;
	maxClusterIdx.push_back(0);
	int maxClusterSize = int(cluster[0].size());

	// changed code to sort the clusters in descending order, not just selecting maximum sized cluster
	std::sort(cluster.begin(), cluster.end(), [](const vector<int> & a, const vector<int> & b){ return a.size() > b.size(); });

	//cout << "clusters ordered by their size" << endl;
	return;
}

bool findFrontierMulti::allocateGoalsByDistance(){
	destinationCluster.clear();
	vector<int> allocated_agent_id;
	allocated_agent_id.clear();
	for (int i = 0 ; i < min(num_agents, static_cast<int>(cluster.size())); i++){
		// select median point of cluster
		//cout << "static_cast<int>(cluster[i].size() / 2): " << static_cast<int>(cluster[i].size() / 2) << endl;
		vector<int>::iterator median_it = cluster[i].begin() + static_cast<int>(cluster[i].size() / 2);
		//cout << "median it declared, cluster[" << i << "]'s size: " << cluster[i].size() << endl;
		nth_element(cluster[i].begin(), median_it, cluster[i].end());
		if (cluster[i].size() == 1 ){
			median_it = cluster[i].begin();
		}
		int median = *median_it;
		int cluster_median_x, cluster_median_y;

		cluster_median_x = FrontierList[median]%size_x;
		cluster_median_y = FrontierList[median]/size_x;

		//if (cluster[i].size() == 1){
		//	cluster_median_x = FrontierList[0]%size_x;
		//	cluster_median_y = FrontierList[0]/size_x;
		//}


		//cout << "cluster_median_x, cluster_median_y: " << cluster_median_x << ", " << cluster_median_y << endl;

		vector<double> distanceSq(num_agents, numeric_limits<double>::infinity());
		for(int j = 0; j < num_agents; j++){
			if ( find(allocated_agent_id.begin(), allocated_agent_id.end(), j) != allocated_agent_id.end() ){
				continue;
			}
			distanceSq[j] = abs(int((pose_ptr_->poses[explore_agent_id_list[j]].position.x/map_scale)-0.5)-cluster_median_x)
										  + abs(int(pose_ptr_->poses[explore_agent_id_list[j]].position.y/map_scale-0.5)-cluster_median_y);
			//cout << "distanceSq[" << j << "].x: " << int((pose_ptr_->poses[j].position.x/map_scale)-0.5)-cluster_median_x << ", y: " <<  int(pose_ptr_->poses[j].position.y/map_scale - 0.5)-cluster_median_y << endl;
			//cout << "distanceSq[" << j << "]: " << distanceSq[j] << endl;
		}
		int min_dist_agent_index = min_element(begin(distanceSq), end(distanceSq)) - begin(distanceSq);

		//cout << "min_dist_agent_index: " << min_dist_agent_index << endl;


		double min_dist = *min_element(begin(distanceSq), end(distanceSq)); // this line is just for debug

		//cout << "min_dist: " << min_dist << endl;

		destinationCluster.insert(pair<int, vector<int> >(min_dist_agent_index, cluster[i]));
		allocated_agent_id.push_back(min_dist_agent_index);

	}  // now the algorithm is O(num_agent^2)

	//cout << "destination Cluster generated" << endl;
	int num_remained_agents = num_agents - static_cast<int>(cluster.size());
	points.color.r = 1.0f;
	points.color.a = 1.0;
	for(int i=0;i<int(FrontierList.size());i++){
		float x = float(FrontierList[i]%size_x)*map_scale-map_start_x;
		float y = float(FrontierList[i]/size_x)*map_scale-map_start_y;
		float z = 0.0f;
		geometry_msgs::Point p;
		p.x = x + map_start_x + map_scale*0.5;
		p.y = y + map_start_y + map_scale*0.5;
		p.z = z;
		points.points.push_back(p);
	}

	//cout << "points push-backed" << endl;

	cluster_points.color.g = 1.0f;
	cluster_points.color.a = 1.0;
	for (int t = 0; t <  min(num_agents, static_cast<int>(cluster.size())); t++){
		if (find(allocated_agent_id.begin(), allocated_agent_id.end(), t) == allocated_agent_id.end()){
			continue;
		}
		for(int i=0;i<int(destinationCluster[t].size());i++){
			//cout << "t: " << t << ", i: " << i << ", destinationCluster[t].size(): "
			//		<< destinationCluster[t].size() << ", destinationCluster[t].at(i): "
			//		<< destinationCluster[t].at(i) << endl;
			float x = float(FrontierList.at(destinationCluster[t].at(i))%size_x)*map_scale-map_start_x;
			float y = float(FrontierList.at(destinationCluster[t].at(i))/size_x)*map_scale-map_start_y;
			float z = 0.0f;
			geometry_msgs::Point p;
			p.x = x + map_start_x + map_scale*0.5;
			p.y = y + map_start_y + map_scale*0.5;
			p.z = z;
			cluster_points.points.push_back(p);
		}
	}

	//cout << "cluster points push-backed" << endl;
	// make median(of the array) of the cluster as goal points
	// no-goal allocated agents receives -1, -1
	for (int i = 0; i < num_agents; i++){
		if ( destinationCluster.find(i) == destinationCluster.end()){
			goal_x.push_back(-1);
			goal_y.push_back(-1);
			continue;
		}
		const vector<int>::iterator median_it = destinationCluster[i].begin() + static_cast<int>(destinationCluster[i].size() / 2);
		nth_element(destinationCluster[i].begin(), median_it, destinationCluster[i].end());
		int median = *median_it;
		goal_x.push_back(static_cast<float>(FrontierList[median] % size_x) * map_scale);
		goal_y.push_back(static_cast<float>(FrontierList[median] / size_x) * map_scale);
	}
	return true;
}

void findFrontierMulti::processAndPublishGoals(){
    goal_positions_.header.frame_id = "map";
    goal_positions_.header.stamp = ros::Time::now();

    // make the goal position to the center of the grid
    for (int i = 0; i < num_agents; i++){
    	if (goal_x.at(i) == -1){
    		geometry_msgs::Pose pose;
    		pose.position.x = -1;
    		pose.position.y = -1;
    		pose.orientation.w = goal_w;
    		goal_positions_.poses.push_back(pose);
    		continue;
    	}
		int goal_x_grid = goal_x.at(i) / map_scale;
		goal_x_grid = goal_x_grid * map_scale;
		int goal_y_grid = goal_y.at(i) / map_scale;
		goal_y_grid = goal_y_grid * map_scale;
		geometry_msgs::Pose pose;
		pose.position.x = goal_x_grid + map_scale*0.5;
		pose.position.y = goal_y_grid + map_scale*0.5;
		pose.orientation.w = goal_w;
		goal_positions_.poses.push_back(pose);
	}
    goal_list_pub.publish(goal_positions_);
    ROS_INFO("Sending goal");
    for (int i = 0; i < static_cast<int>(goal_positions_.poses.size()); i++){
        //cout << "goal_x, goal_y: "<< goal_x.at(i)/map_scale << ", " << goal_y.at(i)/map_scale << endl;
    }
    return;
}

void findFrontierMulti::publishEmptyPlan(){
	vector<vector<geometry_msgs::PoseStamped> > plan;
	geometry_msgs::PoseStamped temp_pose;
	vector<geometry_msgs::PoseStamped> plan_elem;
	plan_elem.clear();
	for (int i = 0; i < num_agents; i++){
		plan.push_back(plan_elem);
	}
	publishPlan(plan);
	makeActionPlan(plan, NULL);
	return;
}

bool findFrontierMulti::allocateGoalsByPlan(){
	/* 1. Generate table for agents which has real distance to each cluster by plan size
	 * 2. Allocate goals for each agent which has min distance, if it is not in allocated_cluster_id_list
	 * 3. add the allocated cluster's id to allocated_cluster_id_list
	*/

	/* 1. Generate table for agents which has real distance to each cluster by plan size*/
	vector< vector<int> > distanceTable;
	vector<int> distanceTable_elem;
	vector<int> allocated_cluster_id;
	destinationCluster.clear();
	goal_x.clear();
	goal_y.clear();
	goal_positions_.poses.clear();
	int fail_counter = 0;
	for (int i = 0; i < num_agents; i++){
		distanceTable_elem.clear();
		for (int j = 0; j < cluster.size(); j++){
			// select median point of cluster
			const vector<int>::iterator median_it = cluster[j].begin() + static_cast<int>(cluster[j].size() / 2);

			//cout << "median it declared, cluster[" << j << "]'s size: " << cluster[j].size() << endl;

			nth_element(cluster[j].begin(), median_it, cluster[j].end());
			int median = *median_it;
			int cluster_median_x = FrontierList[median]%size_x;
			int cluster_median_y = FrontierList[median]/size_x;

			//cout << "cluster_median_x, cluster_median_y: " << cluster_median_x << ", " << cluster_median_y << endl;
			vector<geometry_msgs::PoseStamped> plan;
			geometry_msgs::Pose goal;
			goal.position.x = static_cast<float>((cluster_median_x + 0.5) * map_scale);
			goal.position.y = static_cast<float>((cluster_median_y + 0.5) * map_scale);
			goal.position.z = 0.0;
			int plan_size = getPlanAndSize(goal, plan, i);

			if (plan_size < 0 || find(allocated_cluster_id.begin(), allocated_cluster_id.end(), j) != allocated_cluster_id.end()){
				plan_size = numeric_limits<int>::max();
			}
			distanceTable_elem.push_back(plan_size);
		}
		distanceTable.push_back(distanceTable_elem);
		int min_dist_cluster_index = min_element(begin(distanceTable.at(i)), end(distanceTable.at(i))) - begin(distanceTable.at(i));
		//cout << "agent " << i << "'s min_dist_cluster_index: " << min_dist_cluster_index << endl;
		double min_dist = *min_element(begin(distanceTable.at(i)), end(distanceTable.at(i))); // this line is just for debug
		//cout << "min_dist: " << min_dist << endl;

		if (min_dist == numeric_limits<int>::max()){
			goal_x.push_back(-1);
			goal_y.push_back(-1);
			fail_counter++;
		} else {
			destinationCluster.insert(pair<int, vector<int> >(i, cluster[min_dist_cluster_index]));
			allocated_cluster_id.push_back(min_dist_cluster_index);

			const vector<int>::iterator median_it = cluster[min_dist_cluster_index].begin() + static_cast<int>(cluster[min_dist_cluster_index].size() / 2);
			nth_element(cluster[min_dist_cluster_index].begin(), median_it, cluster[min_dist_cluster_index].end());
			int median = *median_it;

			goal_x.push_back(static_cast<float>(FrontierList[median] % size_x) * map_scale);
			goal_y.push_back(static_cast<float>(FrontierList[median] / size_x) * map_scale);
		}
	}
	if (fail_counter == num_agents){
		return false;
	}
	return true;
}

int findFrontierMulti::getMinCostInGraph(Vertex* vertex, Vertex** src, Vertex** dst){
	Vertex* min_src;
	Vertex* min_dst;
	if (vertex == NULL){
		*src = NULL;
		*dst = NULL;
		return -1;
	}
	if (vertex->m_cost.empty()){
		*src = NULL;
		*dst = NULL;
		return -1;
	}
	int min_cost = vertex->m_cost.begin()->second;
	//cout << "cost from " << vertex->m_type_id << " to " << vertex->m_cost.begin()->first->m_type_id << ": " << min_cost << endl;
	*src = vertex;
	*dst = vertex->m_cost.begin()->first;
	min_src = *src;
	min_dst = *dst;
	int cost;

	for (int i = 0; i < vertex->m_child.size(); i++){
		cost = getMinCostInGraph(vertex->m_child[i], src, dst);
		if (cost == -1){
			continue;
		}
		else if(cost < min_cost){
			min_cost = cost;
			min_src = *src;
			min_dst = *dst;
		}
	}
	*src = min_src;
	*dst = min_dst;
	//cout << "src: " << (*src)->m_type_id << ", dst: " << (*dst)->m_type_id << endl;

	return min_cost;
}

void findFrontierMulti::printMST(Vertex* node){
	if (node == NULL){
		cout << "printMST in NULL vertex" << endl;
		return;
	}
	cout << node->m_type_id << " " << endl;;
	for (int i = 0; i < node->m_child.size(); i++){
		printMST(node->m_child[i]);
	}
	return;
}

void findFrontierMulti::printMST(){
	for (int i = 0; i < m_num_planning_agent; i++){
		cout << "Root " << i << " MST: " << endl;;
		printMST(&m_root[i]);
	}
}

void findFrontierMulti::generateGraph(find_frontier::PlanningID::Request &req){
	m_root.clear();
	m_tasks.clear();

	find_frontier::InitPosList::Request temp_req;
	find_frontier::InitPosList::Response temp_res;

	m_num_planning_agent = req.agent_id_list.size();
	m_num_tasks = req.box_id_list.size();


	geometry_msgs::Pose start;
	geometry_msgs::Pose goal;
	std::vector<geometry_msgs::PoseStamped> plan;
	m_root.reserve(m_num_planning_agent);
	for (int i = 0; i < m_num_planning_agent; i++){
		m_root.push_back(Vertex(string("r")+to_string(req.agent_id_list[i]), req.agent_pos_x_list[i], req.agent_pos_y_list[i]));
		//temp_req.x.push_back(req.agent_pos_x_list[i]);
		//temp_req.y.push_back(req.agent_pos_y_list[i]);
		//temp_req.dir.push_back(req.agent_dir_list[i]);
		//temp_req.agent_id_list.push_back(req.agent_id_list[i]);
	}
//	PoseListUpdate(temp_req, temp_res);
	cout << "m_num_tasks: " << m_num_tasks << endl;
	if (m_num_tasks == 0){
		cout << "nothing to allocate" << endl;
		return;
	}
	// generate task vertices & connect each other
	m_tasks.reserve(m_num_tasks);
	for (int i = 0; i < m_num_tasks; i++){
		m_tasks.push_back(Vertex(string("t")+to_string(req.box_id_list[i]), req.box_pos_x_list[i], req.box_pos_y_list[i]));
	}

	for (int i = 0; i < m_num_tasks; i++){
		start.position.x = m_tasks[i].m_x;
		start.position.y = m_tasks[i].m_y;
		cout << m_tasks[i].m_type_id << " position: " << (m_tasks[i].m_x)/map_scale - 0.5 <<"," << (m_tasks[i].m_y)/map_scale - 0.5 << endl;
		for (int j = 0; j < m_num_tasks; j++){
			if (j == i) continue;
			goal.position.x = m_tasks[j].m_x;
			goal.position.y = m_tasks[j].m_y;
			int cost = getPlanAndSize(goal, start, plan);
			if (cost < 0){
				continue;
			}
			//cout << m_tasks[i].m_type_id << " to " << m_tasks[j].m_type_id << " cost: " << cost << endl;
			m_tasks[i].addEdge(&m_tasks[j], cost);
			//m_tasks[j].printEdge();
			//cout << "i: " << i << ", j: " << j << endl;
			//for (int k = 0; k < m_tasks.size(); k++){
			//	m_tasks[k].printEdge();
			//}
		}
	}

	// generate roots of tree == robot vertices, and construct a directed edges from robot to tasks
	for (int i = 0; i < m_num_planning_agent; i++){
		start.position.x = m_root[i].m_x;
		start.position.y = m_root[i].m_y;
		for (int j = 0; j < m_num_tasks; j++){
			goal.position.x = m_tasks[j].m_x;
			goal.position.y = m_tasks[j].m_y;
			int cost = getPlanAndSize(goal, start, plan);
			if (cost < 0){
				cost = numeric_limits<int>::max();
				continue;
			}
			m_root[i].addEdge(&m_tasks[j], cost);
			//cout << m_root[i].m_type_id << " to " << m_tasks[j].m_type_id << " cost: " << cost << endl;
		}
	}
	//PoseListUpdate(temp_req, temp_res);
}

void findFrontierMulti::removeTaskFromGraph(string task_id){
	Vertex a(task_id);
	vector<Vertex>::iterator iter = find_if(m_tasks.begin(), m_tasks.end(), Vertex::isTaskId(a));
	if (iter == m_tasks.end()){
		cout << "No such task id in task list" << endl;
		return;
	}
	set<pair<Vertex*, int> >::iterator cost_iter;

	//(*iter).printEdge();
	// remove edges between tasks first
	for (cost_iter = iter->m_cost.begin(); iter->m_cost.size() > 0; cost_iter = iter->m_cost.begin()){
		//cout << "removeEdge from " << iter->m_type_id << " to " << cost_iter->first->m_type_id << endl;
		iter->removeEdge(cost_iter->first);
		//cout << "iter->m_cost.size(): " << iter->m_cost.size() << endl;
	}
	//cout << "after first for loop" << endl;
	// remove edges from robot to the task
	for (int i = 0; i < m_num_planning_agent; i++){
		//cout << "removeEdge from " << m_root[i].m_type_id.c_str() << " to " << iter->m_type_id.c_str() << endl;
		m_root[i].removeEdge(&(*iter));
	}
}

bool findFrontierMulti::allocateTask(find_frontier::PlanningID::Request& req,
		find_frontier::PlanningID::Response &res){
	// To check data properly
	cout << "req.agent_id_list: ";
	for (int i = 0; i < req.agent_id_list.size(); i++){
		cout << req.agent_id_list[i] << " ";
	}
	cout << endl;

	// construct graph first
	//cout << "entered generate graph" << endl;
	generateGraph(req);
	//for (int i = 0; i < m_tasks.size(); i++){
	//	m_tasks[i].printEdge();
	//}
	//cout << "complete generate graph" << endl;

	vector<string> allocated_task_id_list;

	// algorithm from PRIM ALLOCATION from https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1389434
	while(m_num_tasks > allocated_task_id_list.size()){
		m_min_cost.clear();
		Vertex* src[m_num_planning_agent];
		Vertex* dst[m_num_planning_agent];


		// for each robot i, calculate min cost from currently spanning tree to non-allocated tasks
		for (int i = 0; i < m_num_planning_agent; i++){
			int cost = getMinCostInGraph(&m_root.at(i), &src[i], &dst[i]);
			if (cost < 0){
				cost = numeric_limits<int>::max();
			}
			m_min_cost.push_back(cost);
			if (src[i] != NULL && dst[i] != NULL){
				//cout << "src[" << i << "]->m_type_id: " << src[i]->m_type_id << endl;
				//cout << "dst[" << i << "]->m_type_id: " << dst[i]->m_type_id << endl;
			}
			cout << "m_min_cost[" << i << "]: " << m_min_cost[i] << endl;
		}
		cout << "Calculated min_cost for each spanning tree" << endl;


		// find which tree has min cost among them
		int j = min_element(begin(m_min_cost), end(m_min_cost)) - begin(m_min_cost);
		cout << "Found min-cost tree: " << j << endl;
		if (m_min_cost[j] == numeric_limits<int>::max()){
			break;
		}

		// find the task which gives the min cost to above tree
		string task_id_str;
		task_id_str.assign(dst[j]->m_type_id);
		cout << "dst[" << j << "]->m_type_id: " << dst[j]->m_type_id << endl;

		// Attach the task to the spanning tree
		src[j]->m_child.push_back(dst[j]);
		//cout << "Task added to spanning tree" << endl;

		// Remove the allocated task from the graph
		allocated_task_id_list.push_back(task_id_str);
		removeTaskFromGraph(task_id_str);
		//cout << "Removed allocated task" << endl;

		cout << "m_num_tasks: " << m_num_tasks << ", allocated_task_id_list.size(): " << allocated_task_id_list.size() << endl;
	}
	printMST();
	m_planning_mode = req.planning_mode;
	generatePlanFromMST(res);

	return true;
}

int findFrontierMulti::makePlanBeforeBox(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
		vector<geometry_msgs::PoseStamped>& plan, std_msgs::Int32MultiArray& index_plan,
		std_msgs::Int32MultiArray& action_plan, int id){
	if (planner_costmap_ros_ == NULL){
		ROS_ERROR("makePlanBeforeBox: Planner costmap ROS is NULL, unable to create global plan");
		return -1;
	}
	if (!planner_[0]->makePlan(start, goal, plan) || plan.empty()){
		ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
		return -1;
	}
	makeActionPlan(plan, index_plan, action_plan, id);
	if (!action_plan.data.empty() && action_plan.data.at(action_plan.data.size()-1) == static_cast<int>(findFrontierMulti::Action::FWD)){
		action_plan.data.pop_back();
	}
	return 0;
}

void findFrontierMulti::makeActionPlan(const std::vector<geometry_msgs::PoseStamped> plan,
		std_msgs::Int32MultiArray& index_plan, std_msgs::Int32MultiArray& action_plan, int id){
	if (plan.size() == 0){
		//cout << "empty plan for agent " << k << endl;
		action_plan.data.push_back(-1);
		return;
	}
	vector<geometry_msgs::PoseStamped> grid_plan;
	int index_plan_start = index_plan.data.size() - 1;
	int index = findGridIndex(plan.at(0));
	//index_plan.data.push_back(index);
	/**********************************************************************
	* indexing & grid_plan pose coordinating is done in the following order
	* -----> x 0, 1, 2,..., size_x-1
	* |        size_x, size_x+1, ...,
	* |
	* v
	* y
	*
	* Caution: in ros msg,
	* ^ y
	* |
	* |         size_x, ...,
	* ------->x 0, 1, 2, ..., size_x-1
	* Thus, index is modified before indexing to ros msg
	************************************************************************/

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = static_cast<int>(index % size_x);
	pose.pose.position.y = static_cast<int>(index / size_x);
	grid_plan.push_back(pose);
	int prev_index = index;
	for (int i = 1; i < plan.size(); i++){
		index = findGridIndex(plan.at(i));
		if (index == prev_index) continue;
		else {
			index_plan.data.push_back(index);
			pose.pose.position.x = static_cast<int>(index % size_x);
			pose.pose.position.y = static_cast<int>(index / size_x);
			grid_plan.push_back(pose);
			prev_index = index;
		}
	}

	if (grid_plan.size() < 2){
		action_plan.data.push_back(rand() % 3);
		return;
	}

	vector<geometry_msgs::PoseStamped>::iterator it, prev_it;
	vector<int>::iterator index_it, index_prev_it;
	prev_it = grid_plan.begin();
	index_prev_it = index_plan.data.begin() + index_plan_start;

	it = next(prev_it, 1);
	index_it = next(index_prev_it, 1);
	int dx,dy, dth;
	for (; it != grid_plan.end(); it++){
		prev_index = *index_prev_it;
		index = *index_it;
		//cout << "next pose: " << it->pose.position.x << "," <<it->pose.position.y << ", curr_pose: " << prev_it->pose.position.x << "," <<prev_it->pose.position.y << endl;
		dx = it->pose.position.x - prev_it->pose.position.x;
		dy = it->pose.position.y - prev_it->pose.position.y;
		//cout << "dx, dy: " << dx << "," << dy << endl;
		unsigned int mx = 0;
		unsigned int my = 0;
		unsigned char cost;

		if (dx == 0 or dy == 0){
			if (dy == 0){
				if (dx > 0){ // should go right
					addGoRightAction(action_plan, &agent_dir[id]);
				} else if (dx < 0){ // should go left
					addGoLeftAction(action_plan, &agent_dir[id]);
				}
			} else if (dx == 0){
				if (dy > 0){ // should go down
					addGoDownAction(action_plan, &agent_dir[id]);
				} else if (dy < 0){ // should go up
					addGoUpAction(action_plan, &agent_dir[id]);
				}
			}

		} else if (dx > 0){
			index = index % size_x + static_cast<int>(size_y - 1 - index / size_x) * size_x;
			prev_index = prev_index % size_x + static_cast<int>(size_y - 1 - prev_index / size_x) * size_x;
			//cout << "index: " << index << ", prev_index: " << prev_index << endl;
			if (dy > 0){
				planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(index - 1), mx, my);
				cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);
				if (cost == costmap_2d::FREE_SPACE){ // go down first, then go right
					// go down
					addGoDownAction(action_plan, &agent_dir[id]);
					addGoRightAction(action_plan, &agent_dir[id]);
				} else{
					planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(prev_index + 1), mx, my);
					cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);
					if (cost == costmap_2d::FREE_SPACE){ // go right first
						addGoRightAction(action_plan, &agent_dir[id]);
						addGoDownAction(action_plan, &agent_dir[id]);
					} else {
						//cout << "findFrontierMulti: makeActionPlan: dx>0 dy>0: This should not happen" << endl;
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index - 1]: " << static_cast<int>(msg->data[index-1]) << ", msg->data[prev_index + 1]: " << static_cast<int>(msg->data[prev_index + 1]) << endl;
					}
				}

			} else { // dy < 0
				planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(index - 1), mx, my);
				cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);
				if (cost == costmap_2d::FREE_SPACE){ // go up then go right
					addGoUpAction(action_plan, &agent_dir[id]);
					addGoRightAction(action_plan, &agent_dir[id]);
				} else{
					planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(prev_index + 1), mx, my);
					cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);

					if (cost == costmap_2d::FREE_SPACE){
						addGoRightAction(action_plan, &agent_dir[id]);
						addGoUpAction(action_plan, &agent_dir[id]);
					} else {
						//cout << "msg->data[index - 1]: " << static_cast<int>(msg->data[index-1]) << ", msg->data[prev_index + 1]: " << static_cast<int>(msg->data[prev_index + 1]) << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx>0, dy<0 This should not happen" << endl;
					}
				}
			}
		} else if (dx < 0){
			index = index % size_x + static_cast<int>(size_y- 1 - index / size_x) * size_x;
			prev_index = prev_index % size_x + static_cast<int>(size_y - 1 - prev_index / size_x) * size_x;
			//cout << "index: " << index << ", prev_index: " << prev_index << endl;

			if (dy > 0){
				planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(index + 1), mx, my);
				cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);

				if (cost == costmap_2d::FREE_SPACE){ // go down first, then go left
					// go down
					addGoDownAction(action_plan, &agent_dir[id]);
					addGoLeftAction(action_plan, &agent_dir[id]);
				} else {
					planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(prev_index - 1), mx, my);
					cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);

					if (cost == costmap_2d::FREE_SPACE){ // go left first
						addGoLeftAction(action_plan, &agent_dir[id]);
						addGoDownAction(action_plan, &agent_dir[id]);
					} else {
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index + 1]: " << static_cast<int>(msg->data[index+1]) << ", msg->data[prev_index - 1]: " << static_cast<int>(msg->data[prev_index - 1]) << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx<0 dy>0: This should not happen" << endl;
					}
				}
			} else { // dy < 0
				planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(index + 1), mx, my);
				cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);

				if (cost == costmap_2d::FREE_SPACE){ // go up then go left
					addGoUpAction(action_plan, &agent_dir[id]);
					addGoLeftAction(action_plan, &agent_dir[id]);
				} else{
					planner_costmap_ros_[0]->getCostmap()->indexToCells(static_cast<unsigned int>(prev_index - 1), mx, my);
					cost = planner_costmap_ros_[0]->getCostmap()->getCost(mx, my);

					if (cost == costmap_2d::FREE_SPACE){
						addGoLeftAction(action_plan, &agent_dir[id]);
						addGoUpAction(action_plan, &agent_dir[id]);
					} else {
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index + 1]: " << msg->data[index+1] << ", msg->data[prev_index - 1]: " << msg->data[prev_index - 1] << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx<0, dy<0 This should not happen" << endl;
					}
				}
			}
		}
		prev_it = it;
		index_prev_it = index_it;
		index_it++;
	}
}

void findFrontierMulti::generatePlanFromMST(Vertex* node, vector<geometry_msgs::PoseStamped>& plan,
		std_msgs::Int32MultiArray& index_plan, std_msgs::Int32MultiArray& action_plan, int id){
	if (node == NULL){
		return;
	}
	for (int i = 0; i < node->m_child.size(); i++){
		geometry_msgs::PoseStamped start, goal;
		goal.header.frame_id = "map";
		goal.header.stamp = ros::Time::now();
		goal.pose.position.x = node->m_child[i]->m_x;
		goal.pose.position.y = node->m_child[i]->m_y;
		int start_idx = index_plan.data.at(index_plan.data.size()-1);
		start.pose.position.x = (start_idx % size_x + 0.5) * map_scale;
		start.pose.position.y = (size_y - static_cast<int>(start_idx / size_x) - 0.5) * map_scale;
		start.header.frame_id = "map";
		start.header.stamp = ros::Time::now();
		makePlanBeforeBox(start, goal, plan, index_plan, action_plan, id);
		if (m_planning_mode == find_frontier::PlanningID::Response::SEARCH){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::OPEN));
		} else if (m_planning_mode == find_frontier::PlanningID::Response::KEEP){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::PICKUP));
		} else if (m_planning_mode == find_frontier::PlanningID::Response::BRING){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::DROP));
		}
		generatePlanFromMST(node->m_child[i], plan, index_plan, action_plan, id);
	}
}

void findFrontierMulti::generatePlanFromMST(find_frontier::PlanningID::Response &res){
	vector<vector<geometry_msgs::PoseStamped> > plan_list;
	for (int i = 0; i < m_num_planning_agent; i++){
		vector<geometry_msgs::PoseStamped> plan;
		std_msgs::Int32MultiArray index_plan, action_plan;
		index_plan.data.clear();
		plan.clear();
		geometry_msgs::PoseStamped robot_pose;
		robot_pose.pose.position.x = m_root[i].m_x;
		robot_pose.pose.position.y = m_root[i].m_y;
		index_plan.data.push_back(findGridIndex(robot_pose));
		plan.push_back(robot_pose);
		string id_str;
		id_str.assign(m_root[i].m_type_id);
		id_str.erase(0,1);
		int id = stoi(id_str);
		generatePlanFromMST(&m_root[i], plan, index_plan, action_plan, id);

		// indicate no plan for this robot
		if (action_plan.data.size() == 0){
			action_plan.data.push_back(-1);
		}
		res.action_array.list.push_back(action_plan);
		plan_list.push_back(plan);
	}
}

void findFrontierMulti::mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	if (m_is_planning){
		size_x = msg->info.width; // # of grid
		size_y = msg->info.height;
		return;
	}
	selectFrontierPoints(msg);
	if (FrontierList.size() != 0){
		clusterFrontierPoints();
		bool allocate_success = allocateGoalsByDistance();
		bool reallocate_success;
		int replan_fail_num;
		marker_pub.publish(points);
		marker_pub.publish(cluster_points);
		processAndPublishGoals();

		int plan_fail_num = makePlan(goal_positions_, *planner_plan_);
		if (!allocate_success || plan_fail_num == -1 || plan_fail_num > 0){
			reallocate_success = allocateGoalsByPlan();
			processAndPublishGoals();
			replan_fail_num = makePlan(goal_positions_, *planner_plan_);
			if (!reallocate_success || replan_fail_num == -1 || replan_fail_num == num_agents){
				publishEmptyPlan();
				m_is_planning = true;
				return;
			}
		}
	    makeActionPlan(*planner_plan_, msg);
	} else{
	    ROS_INFO("No Frontier Detected");
		publishEmptyPlan();
	}
	m_is_planning = true;
	return;
}

void findFrontierMulti::addGoRightAction(std_msgs::Int32MultiArray& action_plan, int *dir){
	int dth;
	dth = *dir - static_cast<int>(findFrontierMulti::Direction::EAST);
	*dir = static_cast<int>(findFrontierMulti::Direction::EAST);
	if (dth < 0) dth += 4; // to make range from 0 to 3
	if (dth == 3) {
		action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::RIGHT));
	} else {
		for (int i = 0; i < dth; i++){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::LEFT));
		}
	}
	action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::FWD));
}


void findFrontierMulti::addGoLeftAction(std_msgs::Int32MultiArray& action_plan, int *dir){
	int dth;
	dth = *dir - static_cast<int>(findFrontierMulti::Direction::WEST);
	*dir = static_cast<int>(findFrontierMulti::Direction::WEST);
	if (dth < 0) dth += 4; // to make range from 0 to 3
	if (dth == 3) {
		action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::RIGHT));
	} else {
		for (int i = 0; i < dth; i++){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::LEFT));
		}
	}
	action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::FWD));
}


void findFrontierMulti::addGoUpAction(std_msgs::Int32MultiArray& action_plan, int *dir){
	int dth;
	dth = *dir - static_cast<int>(findFrontierMulti::Direction::NORTH);
	*dir = static_cast<int>(findFrontierMulti::Direction::NORTH);
	if (dth < 0) dth += 4; // to make range from 0 to 3
	if (dth == 3) {
		action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::RIGHT));
	} else {
		for (int i = 0; i < dth; i++){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::LEFT));
		}
	}
	action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::FWD));
}

void findFrontierMulti::addGoDownAction(std_msgs::Int32MultiArray& action_plan, int *dir){
	int dth;
	dth = *dir - static_cast<int>(findFrontierMulti::Direction::SOUTH);
	*dir = static_cast<int>(findFrontierMulti::Direction::SOUTH);
	if (dth < 0) dth += 4; // to make range from 0 to 3
	if (dth == 3) {
		action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::RIGHT));
	} else {
		for (int i = 0; i < dth; i++){
			action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::LEFT));
		}
	}
	action_plan.data.push_back(static_cast<int>(findFrontierMulti::Action::FWD));
}


void findFrontierMulti::makeActionPlan(const vector<vector<geometry_msgs::PoseStamped> > plan, const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//cout << "makeActionPlan entered" << endl;
	std_msgs::Int32MultiArray index_plan, action_plan;
	vector<geometry_msgs::PoseStamped> grid_plan;
	find_frontier::ActionArray action_array;
	for (int k = 0; k < num_agents; k++){
		index_plan.data.clear();
		action_plan.data.clear();
		grid_plan.clear();

		if (plan.at(k).size() == 0){
			//cout << "empty plan for agent " << k << endl;
			action_plan.data.push_back(-1);
			action_array.list.push_back(action_plan);
			continue;
		}
		int index = findGridIndex(plan.at(k).at(0));
		index_plan.data.push_back(index);

		/**********************************************************************
		* indexing & grid_plan pose coordinating is done in the following order
		* -----> x 0, 1, 2,..., size_x-1
		* |        size_x, size_x+1, ...,
		* |
		* v
		* y
		*
		* Caution: in ros msg,
		* ^ y
		* |
		* |         size_x, ...,
		* ------->x 0, 1, 2, ..., size_x-1
		* Thus, index is modified before indexing to ros msg
		************************************************************************/

		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = static_cast<int>(index % size_x);
		pose.pose.position.y = static_cast<int>(index / size_x);
		grid_plan.push_back(pose);
		int prev_index = index;
		for (int i = 1; i < plan.at(k).size(); i++){
			index = findGridIndex(plan.at(k).at(i));
			if (index == prev_index) continue;
			else {
				index_plan.data.push_back(index);
				//cout << index / size_x << endl;
				pose.pose.position.x = static_cast<int>(index % size_x);
				pose.pose.position.y = static_cast<int>(index / size_x);
				grid_plan.push_back(pose);
				prev_index = index;
			}
		}

		if (grid_plan.size() < 2){
		    action_plan.data.push_back(rand() % 3);
		    continue;
		}
		vector<geometry_msgs::PoseStamped>::iterator it, prev_it;
		vector<int>::iterator index_it, index_prev_it;
		prev_it = grid_plan.begin();
		index_prev_it = index_plan.data.begin();

		it = next(prev_it, 1);
		index_it = next(index_prev_it, 1);
		int dx,dy, dth;
		int dir = agent_dir[explore_agent_id_list[k]];
		for (; it != grid_plan.end(); it++){
			prev_index = *index_prev_it;
			index = *index_it;
			//cout << "next pose: " << it->pose.position.x << "," <<it->pose.position.y << ", curr_pose: " << prev_it->pose.position.x << "," <<prev_it->pose.position.y << endl;
			dx = it->pose.position.x - prev_it->pose.position.x;
			dy = it->pose.position.y - prev_it->pose.position.y;
			//cout << "dx, dy: " << dx << "," << dy << endl;
			if (dx == 0 or dy == 0){
				if (dy == 0){
					if (dx > 0){ // should go right
						addGoRightAction(action_plan, &dir);
					} else if (dx < 0){ // should go left
						addGoLeftAction(action_plan, &dir);
					}
				} else if (dx == 0){
					if (dy > 0){ // should go down
						addGoDownAction(action_plan, &dir);
					} else if (dy < 0){ // should go up
						addGoUpAction(action_plan, &dir);
					}
				}

			} else if (dx > 0){
				index = index % size_x + static_cast<int>(size_y - 1 - index / size_x) * size_x;
				prev_index = prev_index % size_x + static_cast<int>(size_y - 1 - prev_index / size_x) * size_x;
				//cout << "index: " << index << ", prev_index: " << prev_index << endl;

				if (dy > 0){
					if (static_cast<int>(msg->data[index - 1]) == 0){ // go down first, then go right
						// go down
						addGoDownAction(action_plan, &dir);
						addGoRightAction(action_plan, &dir);
					} else if (static_cast<int>(msg->data[prev_index + 1]) == 0){ // go right first
						addGoRightAction(action_plan, &dir);
						addGoDownAction(action_plan, &dir);
					} else {
						//cout << "findFrontierMulti: makeActionPlan: dx>0 dy>0: This should not happen" << endl;
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index - 1]: " << static_cast<int>(msg->data[index-1]) << ", msg->data[prev_index + 1]: " << static_cast<int>(msg->data[prev_index + 1]) << endl;
					}
				} else { // dy < 0
					if (static_cast<int>(msg->data[index - 1]) == 0){ // go up then go right
						addGoUpAction(action_plan, &dir);
						addGoRightAction(action_plan, &dir);
					} else if (static_cast<int>(msg->data[prev_index + 1]) == 0){
						addGoRightAction(action_plan, &dir);
						addGoUpAction(action_plan, &dir);
					} else {
						//cout << "msg->data[index - 1]: " << static_cast<int>(msg->data[index-1]) << ", msg->data[prev_index + 1]: " << static_cast<int>(msg->data[prev_index + 1]) << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx>0, dy<0 This should not happen" << endl;
					}
				}
			} else if (dx < 0){
				index = index % size_x + static_cast<int>(size_y- 1 - index / size_x) * size_x;
				prev_index = prev_index % size_x + static_cast<int>(size_y - 1 - prev_index / size_x) * size_x;
				//cout << "index: " << index << ", prev_index: " << prev_index << endl;

				if (dy > 0){
					if (static_cast<int>(msg->data[index + 1]) == 0){ // go down first, then go left
						// go down
						addGoDownAction(action_plan, &dir);
						addGoLeftAction(action_plan, &dir);
					} else if (static_cast<int>(msg->data[prev_index - 1]) == 0){ // go left first
						addGoLeftAction(action_plan, &dir);
						addGoDownAction(action_plan, &dir);
					} else {
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index + 1]: " << static_cast<int>(msg->data[index+1]) << ", msg->data[prev_index - 1]: " << static_cast<int>(msg->data[prev_index - 1]) << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx<0 dy>0: This should not happen" << endl;
					}
				} else { // dy < 0
					if (static_cast<int>(msg->data[index + 1]) == 0){ // go up then go left
						addGoUpAction(action_plan, &dir);
						addGoLeftAction(action_plan, &dir);
					} else if (static_cast<int>(msg->data[prev_index - 1]) == 0){
						addGoLeftAction(action_plan, &dir);
						addGoUpAction(action_plan, &dir);
					} else {
						//cout << "index: " << index << ", prev_index: " << prev_index << endl;
						//cout << "msg->data[index + 1]: " << msg->data[index+1] << ", msg->data[prev_index - 1]: " << msg->data[prev_index - 1] << endl;
						//cout << "findFrontierMulti: makeActionPlan: dx<0, dy<0 This should not happen" << endl;
					}
				}
			}
			prev_it = it;
			index_prev_it = index_it;
			index_it++;
		}
		/************** For debugging *************************
		cout << "agent " << k << endl;
		cout << "index plan: " << endl;
		for (int k = 0; k < index_plan.data.size(); k++){
			cout << "  index: " << index_plan.data[k] << endl;
		}

		cout << "grid plan: "<< endl;
		for (it = grid_plan.begin(); it != grid_plan.end(); it++){
			cout << "  x,y: " << it->pose.position.x << "," << it->pose.position.y << endl;
		}

		cout << "action plan: " << endl;
		for (int k = 0; k < action_plan.data.size(); k++){
			cout << "  action: " << action_plan.data[k] << endl;
		}
		******************************************************/
		action_array.list.push_back(action_plan);
	}
	action_plan_list_pub.publish(action_array);
}

int findFrontierMulti::findGridIndex(const geometry_msgs::PoseStamped position){
	double x = position.pose.position.x;
	double y = position.pose.position.y;
	int grid_x = round(x / map_scale - 0.5);
	//cout << "double type y result: " << size_y - y / map_scale - 0.5 << endl;
	int grid_y = round(size_y - y / map_scale - 0.5 );
	return grid_x + size_x * grid_y;
}

int findFrontierMulti::makePlan(const geometry_msgs::PoseArray& goal, vector<vector<geometry_msgs::PoseStamped> >& plan){
	//cout << "makePlan entered" << endl;
	int fail_counter = 0;
	plan.clear();
	geometry_msgs::PoseStamped temp_pose;
	vector<geometry_msgs::PoseStamped> plan_elem;
	plan_elem.clear();
	for (int i = 0; i < num_agents; i++){
		plan.push_back(plan_elem);
	}
	if (planner_costmap_ros_ == NULL){
		ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
		return -1;
	}

	// get the starting pose of the robot
	for (int i = 0 ; i < num_agents; i++){
		if (goal.poses[i].position.x < 0){
			continue;
		}
		//cout << "agent " << i << "'s start_x: " << int(pose_ptr_->poses[i].position.x / map_scale - 0.5) << endl;

		geometry_msgs::PoseStamped agent_pose, goal_pose;
		agent_pose.header.frame_id = "map";
		agent_pose.header.stamp = ros::Time::now();
		agent_pose.pose = pose_ptr_->poses[explore_agent_id_list[i]];

		goal_pose.header.frame_id = "map";
		goal_pose.header.stamp = ros::Time::now();
		goal_pose.pose = goal.poses[i];
		if (!planner_[explore_agent_id_list[i]]->makePlan(agent_pose, goal_pose, plan.at(i)) || plan.at(i).empty()){
			ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.poses[i].position.x, goal.poses[i].position.y);
			fail_counter++;
			//return false;
		}

	}
	publishPlan(plan);
	if (fail_counter == num_agents){
		return -1;
	}
	return fail_counter;
}

int findFrontierMulti::getPlanAndSize(const geometry_msgs::Pose& goal, const geometry_msgs::Pose& start, std::vector<geometry_msgs::PoseStamped>& plan){
	plan.clear();
	geometry_msgs::PoseStamped temp_pose;
	if (planner_costmap_ros_ == NULL){
		ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
		return -1;
	}

	// get the starting pose of the robot
	if (goal.position.x < 0){
		//cout << "negative goal" << endl;
		return -1;
	}

	geometry_msgs::PoseStamped start_pose, goal_pose;
	start_pose.header.frame_id = "map";
	start_pose.header.stamp = ros::Time::now();
	start_pose.pose = start;

	goal_pose.header.frame_id = "map";
	goal_pose.header.stamp = ros::Time::now();
	goal_pose.pose.position.x = goal.position.x;
	goal_pose.pose.position.y = goal.position.y;
	goal_pose.pose.position.z = goal.position.z;

	if (!planner_[0]->makePlan(start_pose, goal_pose, plan) || plan.empty()){
		ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.position.x, goal.position.y);
		return -1;
	}

	std_msgs::Int32MultiArray index_plan, action_plan;
	vector<geometry_msgs::PoseStamped> grid_plan;
	find_frontier::ActionArray action_array;


	index_plan.data.clear();
	action_plan.data.clear();

	if (plan.size() == 0){
		return 0;
	}
	int index = findGridIndex(plan.at(0));
	index_plan.data.push_back(index);

	/**********************************************************************
	* indexing & grid_plan pose coordinating is done in the following order
	* -----> x 0, 1, 2,..., size_x-1
	* |        size_x, size_x+1, ...,
	* |
	* v
	* y
	*
	* Caution: in ros msg,
	* ^ y
	* |
	* |         size_x, ...,
	* ------->x 0, 1, 2, ..., size_x-1
	* Thus, index is modified before indexing to ros msg
	************************************************************************/
	//cout << "before for loop to process indexPlan in getPlanAndSize" << endl;
	//cout << "plan.size(): " << plan.size() << endl;
	geometry_msgs::PoseStamped pose;
	//cout << "after pose" << endl;

	//cout << "index: " << index << ", size_x: " << size_x << endl;
	pose.pose.position.x = static_cast<int>(index % size_x);
	pose.pose.position.y = static_cast<int>(index / size_x);

	int prev_index = index;
	for (int i = 1; i < plan.size(); i++){
		index = findGridIndex(plan.at(i));
		if (index == prev_index) continue;
		else {
			index_plan.data.push_back(index);
			if (pose.pose.position.x != static_cast<int>(index % size_x) && pose.pose.position.y != static_cast<int>(index / size_x)){
				int dummy = 0; // to calculate index_plan in taxicab metric
				index_plan.data.push_back(dummy);
			}
			pose.pose.position.x = static_cast<int>(index % size_x);
			pose.pose.position.y = static_cast<int>(index / size_x);
			prev_index = index;
		}
	}
	//cout << "index_plan: " << endl;
	//for (int i = 0; i < index_plan.data.size(); i++){
	//	cout << index_plan.data[i] << endl;
	//}
	return static_cast<int>(index_plan.data.size());

}

int findFrontierMulti::getPlanAndSize(const geometry_msgs::Pose& goal, vector<geometry_msgs::PoseStamped>& plan, int id){
	//cout << "getPlanAndSize entered" << endl;
	plan.clear();
	geometry_msgs::PoseStamped temp_pose;
	if (planner_costmap_ros_ == NULL){
		ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
		return -1;
	}

	// get the starting pose of the robot
	if (goal.position.x < 0){
		//cout << "negative goal" << endl;
		return -1;
	}

	geometry_msgs::PoseStamped agent_pose, goal_pose;
	agent_pose.header.frame_id = "map";
	agent_pose.header.stamp = ros::Time::now();
	agent_pose.pose = pose_ptr_->poses[explore_agent_id_list[id]];

	goal_pose.header.frame_id = "map";
	goal_pose.header.stamp = ros::Time::now();
	goal_pose.pose.position.x = goal.position.x;
	goal_pose.pose.position.y = goal.position.y;
	goal_pose.pose.position.z = goal.position.z;

	if (!planner_[explore_agent_id_list[id]]->makePlan(agent_pose, goal_pose, plan) || plan.empty()){
		ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.position.x, goal.position.y);
		return -1;
	}

	std_msgs::Int32MultiArray index_plan, action_plan;
	vector<geometry_msgs::PoseStamped> grid_plan;
	find_frontier::ActionArray action_array;


	index_plan.data.clear();
	action_plan.data.clear();

	if (plan.size() == 0){
		return 0;
	}
	int index = findGridIndex(plan.at(0));
	index_plan.data.push_back(index);

	/**********************************************************************
	* indexing & grid_plan pose coordinating is done in the following order
	* -----> x 0, 1, 2,..., size_x-1
	* |        size_x, size_x+1, ...,
	* |
	* v
	* y
	*
	* Caution: in ros msg,
	* ^ y
	* |
	* |         size_x, ...,
	* ------->x 0, 1, 2, ..., size_x-1
	* Thus, index is modified before indexing to ros msg
	************************************************************************/

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = static_cast<int>(index % size_x);
	pose.pose.position.y = static_cast<int>(index / size_x);
	int prev_index = index;
	for (int i = 1; i < plan.size(); i++){
		index = findGridIndex(plan.at(i));
		if (index == prev_index) continue;
		else {
			index_plan.data.push_back(index);

			pose.pose.position.x = static_cast<int>(index % size_x);
			pose.pose.position.y = static_cast<int>(index / size_x);
			prev_index = index;
		}
	}
	return static_cast<int>(index_plan.data.size());
}

void findFrontierMulti::publishPlan(const vector<vector<geometry_msgs::PoseStamped> >& path){
		//cout << "publishPlan entered" << endl;
		for (int k = 0; k < path.size(); k++){
			nav_msgs::Path gui_path;
			gui_path.poses.resize(path.at(k).size());
			gui_path.header.frame_id = "map";
			gui_path.header.stamp = ros::Time::now();
			for (unsigned int i = 0; i < path.at(k).size(); i++){
				gui_path.poses[i] = path.at(k).at(i);
			}
			g_plan_pub_list[k].publish(gui_path);
		}
	}

bool findFrontierMulti::PoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res)
{
	std::cout << "PoseUpdate is called" << std::endl;
	pose_ptr_->poses.clear();
	pose_ptr_->poses.reserve(total_num_agents);
	pose_ptr_->header.stamp = ros::Time::now();

	pose_ptr_->header.frame_id = "map";
	static tf2_ros::TransformBroadcaster br;
	for (int i = 0; i < req.x.size(); i++){
		geometry_msgs::Pose pose;
		pose.position.x = req.x[i];
		pose.position.y = req.y[i];
		pose.position.z = 0.0;
		agent_dir[i] = req.dir[i];
		double yaw = -(M_PI/2)*req.dir[i];
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		q.normalize();
		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();
		pose.orientation.w = q.getW();
		pose_ptr_->poses.push_back(pose);

		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = (string("base_link") + to_string(i)).c_str();
		transformStamped.transform.translation.x = req.x[i];
		transformStamped.transform.translation.y = req.y[i];
		transformStamped.transform.translation.z = 0.0;
		transformStamped.transform.rotation.x = q.getX();
		transformStamped.transform.rotation.y = q.getY();
		transformStamped.transform.rotation.z = q.getZ();
		transformStamped.transform.rotation.w = q.getW();
		br.sendTransform(transformStamped);
	}

	if (counter == 0){
		counter++;
		for (int i = 0; i < total_num_agents; i++){
			//cout << "creating planner_costmap_ros" << endl;
			planner_costmap_ros_[i] = new costmap_2d::Costmap2DROS((string("global_costmap") + to_string(i)).c_str(), tf_);
			//cout << "costmap memory allocated\n" << endl;

			planner_costmap_ros_[i]->pause();
			//cout << "costmap paused\n" << endl;

			try {
				planner_[i] = bgp_loader_.createInstance("global_planner/GlobalPlanner");
				//cout << "created Instance\n" << endl;

				planner_[i]->initialize(bgp_loader_.getName("global_planner/GlobalPlanner" + to_string(i)), planner_costmap_ros_[i]);
				//cout << "planner init\n" << endl;

			} catch (const pluginlib::PluginlibException& ex) {
				ROS_FATAL("Failed to create %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", "global_planner/GlobalPlanner", ex.what());
				exit(1);
			}
			//cout << "before start" << endl;
			//getting stucked here.. wtf!!!!
			planner_costmap_ros_[i]->start();
			//cout << "planner_costmap_ros_->start passed\n" << endl;
		}
	}
	IsPoseUpdated = 1;
	res.result = true;
	return true;
}
/*
void findFrontierMulti::PoseSubCb(geometry_msgs::PoseStampedConstPtr pose)
{
	std::cout << "PoseUpdate is called" << std::endl;
	pose_ptr_->header.stamp = ros::Time::now();
	pose_ptr_->header.frame_id = "map";
	pose_ptr_->pose.position.x = pose->pose.position.x;
	pose_ptr_->pose.position.y = pose->pose.position.y;
	pose_ptr_->pose.position.z = pose->pose.position.z;
	pose_ptr_->pose.orientation.x = pose->pose.orientation.x;
	pose_ptr_->pose.orientation.y = pose->pose.orientation.y;
	pose_ptr_->pose.orientation.z = pose->pose.orientation.z;
	pose_ptr_->pose.orientation.w = pose->pose.orientation.w;

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = pose->pose.position.x;
	cout << "PoseSubCb: " << pose->pose.position.x << endl;
	transformStamped.transform.translation.y = pose->pose.position.y;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x = pose->pose.orientation.x;
	transformStamped.transform.rotation.y = pose->pose.orientation.y;
	transformStamped.transform.rotation.z = pose->pose.orientation.z;
	transformStamped.transform.rotation.w = pose->pose.orientation.w;
	br.sendTransform(transformStamped);

}
*/

bool findFrontierMulti::InitPoseListUpdate(find_frontier::InitPosList::Request &req, find_frontier::InitPosList::Response& res){
    std::cout << "InitPoseListUpdate is called" << std::endl;
    m_is_planning = false;
	g_initial_pose_ptr->header.stamp = ros::Time::now();
	g_initial_pose_ptr->header.frame_id = "map";
	num_agents = req.agent_id_list.size();
	explore_agent_id_list.clear();
	explore_agent_id_list.reserve(num_agents);
	for (int i = 0; i < total_num_agents; i++){
		geometry_msgs::Pose pose;
		pose.position.x = req.x[i];
		pose.position.y = req.y[i];
		pose.position.z = 0.0;
		agent_dir[i] = req.dir[i];
		double yaw = -(M_PI/2)*agent_dir[i];
		tf2::Quaternion q;
		q.setRPY(0, 0, yaw);
		q.normalize();
		pose.orientation.x = q.getX();
		pose.orientation.y = q.getY();
		pose.orientation.z = q.getZ();
		pose.orientation.w = q.getW();
		g_initial_pose_ptr->poses.push_back(pose);
		if (i < num_agents) {
			explore_agent_id_list.push_back(req.agent_id_list[i]);
		}
	}
	res.result = true;
	//PoseUpdate(g_initial_pose_ptr);
	g_isInit = true;
	return true;
}

void findFrontierMulti::timerCallback(const ros::TimerEvent& event){
//	cout << "timerCallback is called" << endl;
	if (g_isInit == false) return;
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	for (int i = 0; i < num_agents; i++){
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = string("base_link" + to_string(i)).c_str();
		transformStamped.transform.translation.x = pose_ptr_->poses[i].position.x;
		transformStamped.transform.translation.y = pose_ptr_->poses[i].position.y;
		transformStamped.transform.translation.z = 0.0;
		transformStamped.transform.rotation.x = pose_ptr_->poses[i].orientation.x;
		transformStamped.transform.rotation.y = pose_ptr_->poses[i].orientation.y;
		transformStamped.transform.rotation.z = pose_ptr_->poses[i].orientation.z;
		transformStamped.transform.rotation.w = pose_ptr_->poses[i].orientation.w;
		br.sendTransform(transformStamped);
	}

}


