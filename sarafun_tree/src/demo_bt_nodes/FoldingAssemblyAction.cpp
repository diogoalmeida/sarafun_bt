#include <sarafun_tree/demo_bt_nodes/FoldingAssemblyAction.h>

namespace sarafun {
double FoldingAssemblyAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/folding_assembly/timeout", 30.0, timeout);

  return timeout;
}

bool FoldingAssemblyAction::fillGoal(sarafun_assembly::FoldingGoal &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "FoldingAssembly");
  sarafun::FoldingAssemblyAction fold_action(ros::this_node::getName(),
                                             "/sarafun/assembly/folding",
                                             "folding_assembly_action");
  ros::spin();
  return 1;
}
