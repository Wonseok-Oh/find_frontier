#include <find_frontier/find_frontier.h>
int main(int argc, char **argv)
{
   ros::init(argc, argv, "find_frontier_node");
   std::string process_num = std::string(argv[1]);
   std::cout << process_num.c_str() << std::endl;
   ros::NodeHandle nh("~");
   tf2_ros::Buffer buffer;
   tf2_ros::TransformListener tf(buffer);
   findFrontier find_frontier(buffer, process_num);
   ros::spin();
   return 0;
}
