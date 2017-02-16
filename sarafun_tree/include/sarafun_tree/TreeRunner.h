#include <ros/ros.h>
#include <behavior_tree_core/BehaviorTree.h>
#include <behavior_tree_core/ControlNode.h>
#include <sarafun_tree/parse_tree.h>

using namespace BT;
namespace sarafun
{
  /**
    Class that maintains a running behavior tree.

    It allows for starting and stoping a tree on demand.
    It reloads the tree definition when starting.
  **/
  class TreeRunner
  {
  public:
    /**
      @param tick_period The period at which the tree will be tick'ed (ms).
    **/
    TreeRunner(int tick_period);

    /**
      Starts executing the behavior tree. Loads the tree description from a
      JSON definition.

      @param tree_description_path The path of the JSON file describing the tree.
    **/
    bool startTree(std::string tree_description_path);

    /**
      Stops the tree execution.
    **/
    void stopTree();

  private:
    int tick_period_;
    bt_parser::Parser *parser_;
    ControlNode *root_;
    boost::thread *tree_thread_, *draw_thread_;
  };
}
