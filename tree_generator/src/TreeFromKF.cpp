#include <tree_generator/TreeFromKF.hpp>

using json = nlohmann::json;
namespace tree_generator {

  SubTreeFromKF::SubTreeFromKF()
  {
    nh_ = ros::NodeHandle("~");
    has_label_ = false;
  }

  /*
    Replace all hifens in a string with an underscore
  */
  void replaceWithUnderscore(std::string &label)
  {
    for (int i = 0; i < label.size(); i++)
    {
      if (label[i] == '-')
      {
        label[i] = '_';
      }
    }
  }

  bool SubTreeFromKF::loadLabel(std::string label)
  {
    std::string filename;
    has_label_ = false;

    replaceWithUnderscore(label);

    if (!nh_.getParam(ros::this_node::getName() + "/" + label + "/file", filename))
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

  json SubTreeFromKF::createSubTree(std::vector<int> &indices)
  {
    if (!has_label_)
    {
      throw std::logic_error("Called the createSubTree method without a loaded label");
    }

    file_ >> subtree_;

    file_.close();

    if (!subtree_.count("nodes") || !subtree_.count("root")) {
      throw std::logic_error(
          std::string("The subtree file must declare the 'nodes' and root members "
                      "at the base level!"));
    }

    for (auto i = subtree_["nodes"].begin(); i != subtree_["nodes"].end(); i++) {
      subtree_["nodes"][i.key()] = modifyId((json) subtree_["nodes"][i.key()], indices);
    }

    return subtree_;
  }

  json SubTreeFromKF::modifyId(json node, std::vector<int> &indices)
  {
    if (!node.count("id") || !node.count("type"))
    {
      throw std::logic_error(std::string("modifyId called on a json object without the 'id' and 'type' members!"));
    }

    std::string type = node["type"];
    std::string id = node["id"];

    if(type == "Selector") {
      indices[SEL]++;
      node["id"] = id + std::to_string(indices[SEL]);
    } else if (type == "SelectorStar") {
      indices[SELSTAR]++;
      node["id"] = id + std::to_string(indices[SELSTAR]);
    } else if (type == "Sequence") {
      indices[SEQ]++;
      node["id"] = id + std::to_string(indices[SEQ]);
    } else if (type == "SequenceStar") {
      indices[SEQSTAR]++;
      node["id"] = id + std::to_string(indices[SEQSTAR]);
    } else if (type == "Action") {
      indices[ACTION]++;
      node["id"] = id + std::to_string(indices[ACTION]);
    } else if (type == "Condition") {
      indices[CONDITION]++;
      node["id"] = id + std::to_string(indices[CONDITION]);
    } else {
      std::string error_message("Tried to modify id of an unknown node type: ");
      error_message = error_message + type;
      throw std::logic_error(error_message);
    }

    return node;
  }

  TreeFromKF::TreeFromKF()
  {
    indices_.resize(6);
  }

  json TreeFromKF::createTree(const std::vector<sarafun_msgs::KeyframeMsg> &keyframes_list)
  {
    json tree, root_sequence;
    std::vector<std::string> children_id_list;
    std::map<std::string, json> children;
    std::vector<json> children_list;
    std::string id, old_root, new_root;

    tree["root"] = "root_sequence"; // Assuming that the top-most node is always a sequence

    root_sequence["id"] = "root_sequence";
    root_sequence["type"] = "SequenceStar";
    root_sequence["name"] = "SequenceStar";

    for (int i = 0; i < keyframes_list.size(); i++) // cycle through the keyframes and initialize the pre-defined subtrees
    {
      subtree_parser_.loadLabel(keyframes_list[i].label); // TODO: I am assuming an ordered list. Will this always be the case?
      json subtree = subtree_parser_.createSubTree(indices_);
      // HACK: Add the idx to the subtree root to make it unique. Ideally I would check for uniqueness independently of the idx
      std::cout << subtree << std::endl;
      old_root = subtree["root"];
      new_root = subtree["root"].get<std::string>() + std::to_string(keyframes_list[i].idx);
      subtree["nodes"][new_root] = subtree["nodes"][old_root];
      subtree["root"] = new_root;
      subtree["nodes"].erase(old_root);
      // END HACK
      std::cout << subtree << std::endl;
      id = subtree["root"];
      children_id_list.push_back(id);
      addChildren(subtree, children);
    }

    root_sequence["children"] = children_id_list;
    children["root_sequence"] = root_sequence;
    tree["nodes"] = children;

    std::cout << tree << std::endl;

    return tree;
  }

  void TreeFromKF::addChildren(const json &tree, std::map<std::string, json> &children_map)
  {
    for(auto i = tree["nodes"].begin(); i != tree["nodes"].end(); i++)
    {
      children_map[i.key()] = i.value();
    }
    return;
  }
}
