#include <ros/ros.h>
#include <ros/package.h>
#include <tree_generator/TreeFromKF.hpp>
#include <sarafun_msgs/KeyframeList.h>

tree_generator::TreeFromKF *create_json;
std::string tree_name;

/*
  Get a keyframe list and create the respective json file
*/
void keyframeCallback(const sarafun_msgs::KeyframeList::ConstPtr &msg)
{
  try
  {
    json tree = create_json->createTree(*msg);
    std::ofstream file;
    std::string path = ros::package::getPath("tree_generator") + "/data/generated/" + tree_name + ".json";


    file.open(path);
    file << tree.dump(2);
    file.close();
  }
  catch(std::logic_error &e)
  {
    ROS_ERROR("Tried to create a tree, but got the logic error: %s", e.what());
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tree_generator");
  ros::NodeHandle nh;
  std::string kf_topic;
  std::string node_name = ros::this_node::getName();

  if (!nh.getParam(node_name + "/keyframe_topic_name", kf_topic))
  {
    ROS_ERROR("Tree generator started without a keyframe topic name in the parameter server! Aborting (%s/keyframe_topic_name)", node_name.c_str());
    ros::shutdown();
    return -1;
  }

  if (!nh.getParam(node_name + "/name", tree_name))
  {
    ROS_ERROR("Tree generator requires a name for the generated trees! (%s/name)", node_name.c_str());
    ros::shutdown();
    return -1;
  }

  create_json = new tree_generator::TreeFromKF;
  ros::Subscriber keyframe_subscriber = nh.subscribe(kf_topic, 1, keyframeCallback);
  ROS_INFO("Started the tree generator node");
  ros::spin();
  return 0;
}
