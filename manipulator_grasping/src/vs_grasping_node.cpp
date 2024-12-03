/*
    Visual Servoing Grasping Node
    Author: Mohammad Soltanshah
    Email: m.soltanshah1990@gmail.com
*/

#include <vs_grasping_node.h>
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vs_grasping_node");
  VSGrasping::VSGraspingNode vs_grasping_node(argc, argv);
  vs_grasping_node.run();

  return 0;
}