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
    draw_thread_ = 0;
    root_  = new SequenceNode("root"); // Make a permanent root to avoid screwing up the draw method
  }

  /*
    Creates a behavior tree from the input file and executes it.
  */
  bool TreeRunner::startTree()
  {
    ControlNode *tree_root;
    if (parser_ != 0)
    {
      ROS_WARN("startTree called with an active tree running. Will terminate active tree.");
      stopTree();
    }
    parser_ = new bt_parser::Parser(path_);

    try
    {
      tree_root = dynamic_cast<ControlNode *>(parser_->parseTree());

      if (tree_root == nullptr)
      {
        return false;
      }

      root_->AddChild(tree_root);

      if (draw_thread_ == 0)
      {
        ROS_INFO("Start drawing");
        draw_thread_ = new boost::thread(&drawTree, root_);
      }
    }
    catch(BehaviorTreeException &Exception)
    {
      ROS_ERROR("%s", Exception.what());
      return false;
    }

    tree_thread_ = new boost::thread(Execute, root_, tick_period_);

    enableDrawing();
    return true;
  }

  /*
    Stops a running tree
  */
  void TreeRunner::stopTree()
  {
    disableDrawing();
    sleep(1.0); // TODO: Make this deterministic
    if (tree_thread_ != 0)
    {
      if (tree_thread_->joinable())
      {
        ROS_INFO("Stopping a running behavior tree");
        try
        {
          tree_thread_->interrupt();
          tree_thread_->join();
          delete tree_thread_;
        }
        catch (std::exception &e)
        {
          ROS_WARN("Exception when joining the tree thread: %s", e.what());
        }
        tree_thread_ = 0;
      }

      delete parser_;
      parser_ = 0;
    }
    else
    {
      ROS_WARN("stopTree called with no tree to stop");
    }
  }
}
