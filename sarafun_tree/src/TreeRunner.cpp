#include <sarafun_tree/TreeRunner.h>

using namespace BT;
namespace sarafun
{
  /*
    The tree runner class will read the input file in the given path
    and execute it. On request, it will stop and reload the tree with a new
    description.
  */
  TreeRunner::TreeRunner(int tick_period, std::string tree_description_path)
  {
    path_ = tree_description_path;
    tick_period_ = tick_period;
    parser_ = 0;
    tree_thread_ = 0;
    root_ = 0;
  }

  /*
    Creates a behavior tree from the input file and executes it.
  */
  bool TreeRunner::startTree()
  {
    if (parser_ != 0)
    {
      ROS_WARN("startTree called with an active tree running. Will terminate active tree.");
      stopTree();
    }
    parser_ = new bt_parser::Parser(path_);

    try
    {
      root_ = dynamic_cast<ControlNode *>(parser_->parseTree());

      if (root_ == nullptr)
      {
        return false;
      }
    }
    catch(BehaviorTreeException &Exception)
    {
      ROS_ERROR("%s", Exception.what());
      return false;
    }

    tree_thread_ = new boost::thread(Execute, root_, tick_period_);

    return true;
  }

  /*
    Stops a running tree
  */
  void TreeRunner::stopTree()
  {
    if (tree_thread_ != 0)
    {
      if (tree_thread_->joinable())
      {
        ROS_INFO("Stopping a running behavior tree");
        tree_thread_->interrupt();
        tree_thread_->join();
        delete tree_thread_;
        tree_thread_ = 0;
      }

      delete parser_;
      parser_ = 0;

      delete root_;
      root_ = 0;
    }
    else
    {
      ROS_WARN("stopTree called with no tree to stop");
    }
  }
}
