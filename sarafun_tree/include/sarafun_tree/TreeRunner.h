#include <ros/ros.h>
#include <behavior_tree_core/BehaviorTree.h>
#include <behavior_tree_core/ControlNode.h>
#include <sarafun_tree/parse_tree.h>

using namespace BT;
namespace sarafun
{
  class TreeRunner
  {
  public:
    TreeRunner(int tick_period);
    bool startTree(std::string tree_description_path);
    void stopTree();

  private:
    int tick_period_;
    bt_parser::Parser *parser_;
    ControlNode *root_;
    boost::thread *tree_thread_, *draw_thread_;
  };
}
