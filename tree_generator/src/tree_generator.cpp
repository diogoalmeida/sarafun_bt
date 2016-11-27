#include <ros/ros.h>
#include <tree_generator/TreeFromKF.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tree_generator");
  tree_generator::TreeFromKF create_json();
}
