#include <tree_generator/TreeFromKF.hpp>

using json = nlohmann::json;
namespace tree_generator {

  SubTreeFromKF::SubTreeFromKF()
  {
    nh_ = ros::NodeHandle("~");
    has_label_ = false;
  }

  /*
    Makes sure that the given KF label has a matching defined
    subtree. If so, it reads the file that defines the subtree
  */
  bool SubTreeFromKF::loadLabel(std::string label)
  {
    std::string filename;
    has_label_ = false;

    if (!nh_.getParam(label + "/file", filename))
    {
      ROS_ERROR("Got label %s, which is not defined in the tree generator", label.c_str());
      return false;
    }

    std::string path = ros::package::getPath("tree_generator") + "/data/subtrees/" + filename;
    file_.open(path);

    if (file_.good())
    {
      ROS_INFO("Successfully loaded the tree for label %s", label.c_str());
      has_label_ = true;
      return true;
    }

    ROS_ERROR("The filename for the label %s defined in the parameter server does not exist in /data/subtrees!", label.c_str());
    return false;
  }

  /*
    Creates the subtree for the loaded label, adjusting the id's
    based on the given indices. Updates the given indices.
    Throws a logic error if not called with a loaded label.
  */
  json SubTreeFromKF::createSubTree(std::vector<int> &indices)
  {
    if (!has_label_)
    {
      throw logic_error("Called the createSubTree method without a loaded label");
    }

    file_ >> subtree_;

    if (!subtree_.count("root") || !subtree_.count("nodes")) {
      throw std::logic_error(
          std::string("The subtree file must declare 'root' and 'nodes' members "
                      "at the base level!"));
    }

    std::vector<std::string> nodes = subtree["nodes"];
    for (auto i = nodes.begin(); i != nodes.end(); i++) {
      subtree["nodes"][*i] = modifyId(subtree["nodes"][*i]);
    }

    return subtree["nodes"];
  }

  /*
    Changes the id of a node, given the provided indices.
    Updates the indices.
  */
  json SubTreeFromKF::modifyId(json node, std::vector<int> &indices)
  {
    if (!node.count("id") || !node.count("type"))
    {
      throw std::logic_error(std::string("modifyId called on a json object without the 'id' and 'type' members!"));
    }

    std::string type == node["type"];

    if(type == "Selector") {
      indices[SEL]++
      node["id"] = node["id"] + indices[SEL];
    } else if (type == "SelectorStar") {
      indices[SELSTAR]++
      node["id"] = node["id"] + indices[SELSTAR];
    } else if (type == "Sequence") {
      indices[SEQ]++
      node["id"] = node["id"] + indices[SEQ];
    } else if (type == "SequenceStar") {
      indices[SEQSTAR]++
      node["id"] = node["id"] + indices[SEQSTAR];
    } else if (type == "SequenceStar") {
      indices[SEQSTAR]++
      node["id"] = node["id"] + indices[SEQSTAR];
    } else if (type = "Action") {
      indices[ACTION]++
      node["id"] = node["id"] + indices[ACTION];
    } else if (type == "Condition") {
      indices[CONDITION]++
      node["id"] = node["id"] + indices[CONDITION];
    } else {
      std::string error_message("Tried to modify id of an unknown node type: ");
      error_message = error_message + type;
      throw std::logic_error(error_message);
    }

    return node;
  }

  TreeFromKF::TreeFromKF()
  {
    indices.resize(6);
  }
}
