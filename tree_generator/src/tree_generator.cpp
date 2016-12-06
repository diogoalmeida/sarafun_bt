#include <ros/ros.h>
#include <ros/package.h>
#include <tree_generator/TreeFromKF.hpp>
#include <tree_generator/YamlCreator.hpp>
#include <sarafun_msgs/BTGeneration.h>

tree_generator::TreeFromKF *create_json;
std::string tree_name;

/*
  Get a keyframe list and create the respective json file
*/
bool keyframeCallback(sarafun_msgs::BTGeneration::Request &req,
                      sarafun_msgs::BTGeneration::Response &res)
{
  if (req.keyframe_sequence.size() == 0)
  {
    ROS_ERROR("Received an empty keyframe sequence!");
    res.success = false;
    return true;
  }
  try
  {
    std::string label;
    tree_generator::YamlCreator create_yaml;

    for (int i = 0; i < req.keyframe_sequence.size(); i++)
    {
      label = req.keyframe_sequence[i].label;
      tree_generator::replaceWithUnderscore(label);
      create_yaml.addField(label, "idx", req.keyframe_sequence[i].idx);
    }

    json tree = create_json->createTree(req.keyframe_sequence);
    std::ofstream file;
    std::string tree_path = ros::package::getPath("tree_generator") + "/data/generated/" + tree_name + ".json";

    file.open(tree_path);
    file << tree.dump(2);
    file.close();

    std::string params_path = ros::package::getPath("tree_generator") + "/data/generated/" + tree_name + ".yaml";
    create_yaml.writeFile(params_path);

    res.success = true;
    return true;
  }
  catch(std::logic_error &e)
  {
    ROS_ERROR("Tried to create a tree, but got the logic error: %s", e.what());
    return false;
  }
  catch(...)
  {
    ROS_ERROR("Tried to create a tree, but got an unknown error!");
    return false;
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tree_generator");
  ros::NodeHandle nh;
  std::string kf_service;
  std::string node_name = ros::this_node::getName();

  if (!nh.getParam(node_name + "/keyframe_service_name", kf_service))
  {
    ROS_ERROR("Tree generator started without a keyframe service name in the parameter server! Aborting (%s/keyframe_service_name)", node_name.c_str());
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
  ros::ServiceServer tree_generation_service = nh.advertiseService(kf_service, keyframeCallback);
  ROS_INFO("Started the tree generator node");
  ros::spin();
  return 0;
}
