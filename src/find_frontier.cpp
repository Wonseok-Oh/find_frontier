#include "find_frontier/find_frontier.h"

using namespace std;

findFrontier::findFrontier(tf2_ros::Buffer& tf): tf_(tf),
		goal_x(0), goal_y(0), goal_w(1), IsPoseUpdated(0), counter(0), g_isInit(false),
		bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), planner_costmap_ros_(NULL){
		ros::NodeHandle nh;
		//pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 1, boost::bind(&findFrontier::PoseSubCb, this, _1) );
		navigation_map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("navigation_map", 1, &findFrontier::mapConvert, this);
		init_pos_service = nh.advertiseService("init_pose_update", &findFrontier::InitPoseUpdate, this);
		pos_service = nh.advertiseService("pose_update", &findFrontier::PoseUpdate, this);
		//init_pos_sub = nh.subscribe("initial_pose", 1, &findFrontier::InitPoseUpdate, this);
		//sub = nh.subscribe<nav_msgs::OccupancyGrid>("spatial_map", 1, boost::bind(&findFrontier::mapConvert, this, _1) );
		timer = nh.createTimer(ros::Duration(0.2), &findFrontier::timerCallback, this);
		marker_pub = nh.advertise<visualization_msgs::Marker>("FrontierLocationMarker",1);
		goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal",1, true);
		g_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
		action_pub = nh.advertise<std_msgs::Int32MultiArray>("action", 1);
	    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
	    pose_ptr_ = new geometry_msgs::PoseStamped;
	    g_initial_pose_ptr = new geometry_msgs::PoseStamped;
}

findFrontier::~findFrontier(){
	delete planner_plan_;
	delete planner_costmap_ros_;
	planner_->~BaseGlobalPlanner();
	planner_.reset();
}

void findFrontier::mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  cout << "mapConvert is called" << endl;
  visualization_msgs::Marker points;
  points.header.frame_id ="/map";
  points.header.stamp= ros::Time::now();
  points.ns="frontier_points";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w=1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = map_scale;
  points.scale.y = map_scale;

  visualization_msgs::Marker cluster_points;
  cluster_points.header.frame_id ="/map";
  cluster_points.header.stamp= ros::Time::now();
  cluster_points.ns="frontier_cluster_points";
  cluster_points.action = visualization_msgs::Marker::ADD;
  cluster_points.pose.orientation.w=1.0;
  cluster_points.id = 0;
  cluster_points.type = visualization_msgs::Marker::POINTS;
  cluster_points.scale.x = map_scale;
  cluster_points.scale.y = map_scale;
  float map_start_x, map_start_y;
  if (g_isInit == false && IsPoseUpdated == 0) return;
  else{
      //std::cout << "isInit is true" << std::endl;
	  map_start_x = g_initial_pose_ptr->pose.position.x;
	  map_start_y = g_initial_pose_ptr->pose.position.y;
  }
  int size_x = msg->info.width; // # of grid
  int size_y = msg->info.height;
  vector<int> FrontierList;
  //Find the frontiers
  for(int i=0;i<size_x*size_y;i++)
  {
    int x_ic = i%size_x;
    int y_ic = i/size_x;

    float x_c = float(x_ic)*map_scale-map_start_x;
    float y_c = float(y_ic)*map_scale-map_start_y;
    float distance = 2000;
    if(IsPoseUpdated==1)
    {
      //std::cout << "Pose updated" << std::endl;
      distance = sqrt(pow(pose_ptr_->pose.position.x-x_c,2)+pow(pose_ptr_->pose.position.y-y_c,2));
    }
    if(distance > 3)
    {
      if(msg->data[i]==0)
      {
        if(x_ic>2 && x_ic<size_x-3)
        {
          if(y_ic>2 && y_ic<size_x-3)
          {
            int nearones_5by5[25];
            int nearones_3by3[9];
            int count_occupied = 0;
            for(int j=0;j<25;j++)
            {
              nearones_5by5[j]=i+size_x*(j/5-2)+(j%5-2);
            }
            for(int j=0;j<9;j++)
            {
              nearones_3by3[j]=i+size_x*(j/3-1)+(j%3-1);
            }
            int IsFrontier = 0;
            for(int j=0;j<9;j++)
            {
              if(msg->data[nearones_3by3[j]]==-1)
              {
                IsFrontier = 1;
              }
              //Mark as a frontier if unexplored space is near
            }
            if(IsFrontier==1)
            {
              for(int j=0;j<25;j++)
              {
                if(msg->data[nearones_5by5[j]]==100)
                {
                  count_occupied++;
                }
                //Unmark if occupied space is near
              }
            }
            if(IsFrontier==1 && count_occupied <= 9)
            {
              FrontierList.push_back(i);
              std::cout << "frontier index i: " << i << std::endl;
            }
          }
        }
      }
    }
  }

  if(FrontierList.size()!=0)
  {
    goal_x = 0.0;
    goal_y = 0.0;
    goal_w = 1.0;
    ROS_INFO("Frontier Detected!");
    ROS_INFO("Frontier Size : %d", int(FrontierList.size()));
    //Frontier Detection is Done
    vector<vector<int> > FrontierList_temp;
    FrontierList_temp.resize(2);
    for(int i=0;i<FrontierList.size();i++)
    {
      FrontierList_temp[0].push_back(FrontierList[i]);
      FrontierList_temp[1].push_back(i);
    }
    vector<vector<int> > cluster;


    if(FrontierList.size()>1)
    {
      vector<int> individualCluster;
      vector<int> node;
      node.push_back(0);
      individualCluster.push_back(0);
      while(1)
      {
        vector<int> next_node;
        for(int i=0;i<node.size();i++)
        {
          int baseNode_x = FrontierList[node[i]]%size_x;
          int baseNode_y = FrontierList[node[i]]/size_x;

          for(int j=0;j<FrontierList_temp[0].size();)
          {
            int target_x = FrontierList_temp[0][j]%size_x;
            int target_y = FrontierList_temp[0][j]/size_x;


            int distance = pow(target_x-baseNode_x,2)+pow(target_y-baseNode_y,2);

            if(distance<=2 && distance!=0)
            {

              individualCluster.push_back(FrontierList_temp[1][j]);
              next_node.push_back(FrontierList_temp[1][j]);
              FrontierList_temp[0].erase(FrontierList_temp[0].begin()+j);
              FrontierList_temp[1].erase(FrontierList_temp[1].begin()+j);
            }
            else
            {
              j++;
            }
          }
        }
        node.clear();
        node = next_node;
        next_node.clear();
        if(FrontierList_temp[0].size()==0)
        {
          cluster.push_back(individualCluster);
          individualCluster.clear();
          break;
        }
        else
        {
          if(node.size()==0)
          {
            cluster.push_back(individualCluster);
            individualCluster.clear();
            node.push_back(FrontierList_temp[1][0]);
            FrontierList_temp[0].erase(FrontierList_temp[0].begin());
            FrontierList_temp[1].erase(FrontierList_temp[1].begin());
          }
        }
      }
      //Find the biggest cluster
      vector<int> maxClusterIdx;
      maxClusterIdx.push_back(0);
      int maxClusterSize = int(cluster[0].size());
      for(int i=1;i<int(cluster.size());i++)
      {
        if(int(cluster[i].size())>maxClusterSize)
        {
          maxClusterSize = int(cluster[i].size());
          maxClusterIdx.clear();
          maxClusterIdx.push_back(i);
        }
        else if(int(cluster[i].size())==maxClusterSize)
        {
          maxClusterIdx.push_back(i);
        }
      }

      vector<int> destinationCluster;

      if(int(maxClusterIdx.size())==1)
      {
        destinationCluster = cluster[maxClusterIdx[0]];
        ROS_INFO("Checking Problems");
      }
      else
      {
        vector<float> distance2Clusters;

        ROS_INFO("Checking Problems : %d", int(maxClusterIdx.size()));
        for(int i=0;i<int(maxClusterIdx.size());i++)
        {

          float cluster_meanx = 0;
          float cluster_meany = 0;
          for(int j=0;j<int(cluster[maxClusterIdx[i]].size());j++)
          {
            float cluster_x = float(FrontierList[cluster[maxClusterIdx[i]][j]]%size_x);
            float cluster_y = float(FrontierList[cluster[maxClusterIdx[i]][j]]/size_x);
            cluster_meanx = cluster_meanx+cluster_x/float(cluster[maxClusterIdx[i]].size());
            cluster_meany = cluster_meany+cluster_y/float(cluster[maxClusterIdx[i]].size());
          }
          float distanceTemp = sqrt(pow(pose_ptr_->pose.position.x-cluster_meanx,2)+pow(pose_ptr_->pose.position.y-cluster_meany,2));
          distance2Clusters.push_back(distanceTemp);

        }
        int NearestDistanceID = 0;
        float ShortestDistance = distance2Clusters[0];
        for(int i=1;i<int(distance2Clusters.size());i++)
        {
          if(distance2Clusters[i]<ShortestDistance)
          {
            ShortestDistance = distance2Clusters[i];
            NearestDistanceID = i;
          }
        }

        destinationCluster = cluster[maxClusterIdx[NearestDistanceID]];
      }

      points.color.r = 1.0f;
      points.color.a = 1.0;
      for(int i=0;i<int(FrontierList.size());i++)
      {
        float x = float(FrontierList[i]%size_x)*map_scale-map_start_x;
        float y = float(FrontierList[i]/size_x)*map_scale-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x + map_start_x + map_scale*0.5;
        p.y = y + map_start_y + map_scale*0.5;
        p.z = z;
        points.points.push_back(p);
      }
      cluster_points.color.g = 1.0f;
      cluster_points.color.a = 1.0;
      for(int i=0;i<int(destinationCluster.size());i++)
      {
        float x = float(FrontierList[destinationCluster[i]]%size_x)*map_scale-map_start_x;
        float y = float(FrontierList[destinationCluster[i]]/size_x)*map_scale-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x + map_start_x + map_scale*0.5;
        p.y = y + map_start_y + map_scale*0.5;
        p.z = z;
        cluster_points.points.push_back(p);
        goal_x = goal_x+(float(FrontierList[destinationCluster[i]]%size_x)*map_scale)/(float(destinationCluster.size()));
        goal_y = goal_y+(float(FrontierList[destinationCluster[i]]/size_x)*map_scale)/(float(destinationCluster.size()));
      }
    }
    else
    {
      ROS_INFO("Only one frontier");
      points.color.r = 1.0f;
      points.color.a = 1.0;
      for(int i=0;i<FrontierList.size();i++)
      {
        float x = float(FrontierList[i]%size_x)*map_scale-map_start_x;
        float y = float(FrontierList[i]/size_x)*map_scale-map_start_y;
        float z = 0.0f;
        geometry_msgs::Point p;
        p.x = x + map_start_x + map_scale*0.5;
        p.y = y + map_start_y + map_scale*0.5;
        p.z = z;
        points.points.push_back(p);
        goal_x = goal_x+(float(FrontierList[i]%size_x)*map_scale)/(float(FrontierList.size()));
        goal_y = goal_y+(float(FrontierList[i]/size_x)*map_scale)/(float(FrontierList.size()));
      }

    }
    FrontierList.clear();
    IsPoseUpdated=0;
    ROS_INFO("Cluster size = %d",int(cluster.size()));

    goal_position_.header.frame_id = "map";
    goal_position_.header.stamp = ros::Time::now();
    goal_position_.pose.position.x = goal_x + map_scale*0.5;
    goal_position_.pose.position.y = goal_y + map_scale*0.5;
    goal_position_.pose.orientation.w = goal_w;
    goal_pub.publish(goal_position_);
    ROS_INFO("Sending goal");

    // MakePlan
    makePlan(goal_position_, *planner_plan_);

    // Generate local action from generated plan
    makeActionPlan(*planner_plan_);
  }
  else
  {
    ROS_INFO("No Frontier Detected");
  }
  marker_pub.publish(points);
  marker_pub.publish(cluster_points);
  points.points.clear();
  cluster_points.points.clear();
}

void findFrontier::makeActionPlan(const std::vector<geometry_msgs::PoseStamped> plan){
	std_msgs::Int32MultiArray action_list;
	std::vector<int> index_plan;
	int index = findGridIndex(plan.at(0));
	index_plan.push_back(index);
	int prev_index = index;
	for (int i = 1; i < plan.size(); i++){
		index = findGridIndex(plan.at(i));
		if (index == prev_index) continue;
		else {
			index_plan.push_back(index);
			prev_index = index;
		}
	}

	int current_x = (pose_ptr_->pose.position.x - map_scale * 0.5) / map_scale;
	// assume 4-dim only: up, right, left, down
	prev_index = index_plan.at(0);
	int prev_x, prev_y, x, y;
	for (int i = 1; i < index_plan.size(); i++){
		index = index_plan.at(i);
		indexToXY(prev_index, prev_x, prev_y);
		indexToXY(index, x, y);
		if (x > prev_x){ // should go right

		}
	}
	action_pub.publish(action_list);
}


bool findFrontier::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
	cout << "makePlan entered" << endl;
	plan.clear();
	if (planner_costmap_ros_ == NULL){
		ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
		return false;
	}

	// get the starting pose of the robot
	cout << "x: " << pose_ptr_->pose.position.x << endl;
	if (!planner_->makePlan(*pose_ptr_, goal, plan) || plan.empty()){
		ROS_DEBUG_NAMED("global_planner", "Failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
		return false;
	}
	publishPlan(plan);
	return true;
}

void findFrontier::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
		cout << "publishPlan entered" << endl;
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

bool findFrontier::PoseUpdate(find_frontier::InitPos::Request &req, find_frontier::InitPos::Response& res)
{
	std::cout << "PoseUpdate is called" << std::endl;
	pose_ptr_->header.stamp = ros::Time::now();

	pose_ptr_->header.frame_id = "map";
	pose_ptr_->pose.position.x = req.x;
	pose_ptr_->pose.position.y = req.y;
	pose_ptr_->pose.position.z = 0.0;
	double yaw = -(M_PI/2)*req.dir;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	q.normalize();
	pose_ptr_->pose.orientation.x = q.getX();
	pose_ptr_->pose.orientation.y = q.getY();
	pose_ptr_->pose.orientation.z = q.getZ();
	pose_ptr_->pose.orientation.w = q.getW();

	IsPoseUpdated = 1;
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = req.x;
	transformStamped.transform.translation.y = req.y;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x = q.getX();
	transformStamped.transform.rotation.y = q.getY();
	transformStamped.transform.rotation.z = q.getZ();
	transformStamped.transform.rotation.w = q.getW();
	br.sendTransform(transformStamped);

	if (counter == 0){
		planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
		//cout << "costmap memory allocated\n" << endl;

		//planner_costmap_ros_->pause();
		//cout << "costmap paused\n" << endl;
		counter++;
		try {
			planner_ = bgp_loader_.createInstance("global_planner/GlobalPlanner");
			//cout << "created Instance\n" << endl;

			planner_->initialize(bgp_loader_.getName("global_planner/GlobalPlanner"), planner_costmap_ros_);
			//cout << "planner init\n" << endl;

		} catch (const pluginlib::PluginlibException& ex) {
			ROS_FATAL("Failed to create %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", "global_planner/GlobalPlanner", ex.what());
			res.result = false;
			exit(1);
		}
		cout << "before start" << endl;
		//getting stucked here.. wtf!!!!
		planner_costmap_ros_->start();
		cout << "planner_costmap_ros_->start passed\n" << endl;
	}
	res.result = true;
	return true;
}
/*
void findFrontier::PoseSubCb(geometry_msgs::PoseStampedConstPtr pose)
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

bool findFrontier::InitPoseUpdate(find_frontier::InitPos::Request &req, find_frontier::InitPos::Response& res){
    std::cout << "InitPoseUpdate is called" << std::endl;
	g_initial_pose_ptr->header.stamp = ros::Time::now();
	g_initial_pose_ptr->header.frame_id = "map";
	g_initial_pose_ptr->pose.position.x = req.x;
	g_initial_pose_ptr->pose.position.y = req.y;
	g_initial_pose_ptr->pose.position.z = 0.0;
	double yaw = -(M_PI/2)*req.dir;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	q.normalize();
	g_initial_pose_ptr->pose.orientation.x = q.getX();
	g_initial_pose_ptr->pose.orientation.y = q.getY();
	g_initial_pose_ptr->pose.orientation.z = q.getZ();
	g_initial_pose_ptr->pose.orientation.w = q.getW();
	res.result = true;
	//PoseUpdate(g_initial_pose_ptr);
	g_isInit = true;
	return true;
}

void findFrontier::timerCallback(const ros::TimerEvent& event){
//	cout << "timerCallback is called" << endl;
	if (g_isInit == false) return;
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "base_link";
	transformStamped.transform.translation.x = pose_ptr_->pose.position.x;
	transformStamped.transform.translation.y = pose_ptr_->pose.position.y;
	transformStamped.transform.translation.z = 0.0;
	transformStamped.transform.rotation.x = pose_ptr_->pose.orientation.x;
	transformStamped.transform.rotation.y = pose_ptr_->pose.orientation.y;
	transformStamped.transform.rotation.z = pose_ptr_->pose.orientation.z;
	transformStamped.transform.rotation.w = pose_ptr_->pose.orientation.w;
	br.sendTransform(transformStamped);

}


