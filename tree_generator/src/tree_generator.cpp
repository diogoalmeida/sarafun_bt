#include <ros/ros.h>
#include <ros/package.h>
#include <tree_generator/TreeFromKF.hpp>
#include <sarafun_msgs/KeyframeList.h>

tree_generator::TreeFromKF create_json;

/*
  Get a keyframe list and create the respective json file
*/
void keyframeCallback(const sarafun_msgs::KeyframeList::ConstPtr &msg)
{
  json tree = create_json.createTree(*msg);
  std::ofstream file;
  std::string path = ros::package::getPath("tree_generator") + "/data/generated/test.json";


  file.open(path);
  file << tree;
  file.close();
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tree_generator");
  ros::NodeHandle nh;

  ros::Subscriber keyframe_subscriber = nh.subscribe("/kf_list", 1, keyframeCallback);
  ros::spin();
}
