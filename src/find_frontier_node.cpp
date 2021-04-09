#include <find_frontier/find_frontier.h>
int main(int argc, char **argv)
{
   ros::init(argc, argv, "find_frontier_node");
   tf2_ros::Buffer buffer;
   tf2_ros::TransformListener tf(buffer);
   findFrontier find_frontier(buffer);
   //std::cout << "find_frontier generated" << std::endl;
   ros::spin();
   return 0;
}
