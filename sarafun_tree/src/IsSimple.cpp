#include <sarafun_tree/IsSimple.h>

namespace sarafun {

bool IsSimple::conditionEvaluation() {
  counter++;

  if (counter > condition_limit)
  {
  	return true;
  }
 
  return false;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "IsSimple");
  sarafun::IsSimple is_simple(ros::this_node::getName(),
							  "is_simple_condition");
  ros::spin();
  return 1;
}
