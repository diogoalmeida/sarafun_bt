#include <ros/ros.h>
#include <behavior_tree_core/BehaviorTree.h>
#include <behavior_tree_core/ControlNode.h>
#include <sarafun_tree/ApproachObjectsAction.h>
#include <sarafun_tree/GrabObjectAction.h>
#include <sarafun_tree/FoldingAssemblyAction.h>
#include <sarafun_tree/InsertionWithDeformationAction.h>
#include <sarafun_tree/PlaceAction.h>

using namespace sarafun;
using namespace BT;

int main(int argc, char **argv) {
  ros::init(argc, argv, "sarafun_bt_demo");
  try {
    int TickPeriod_milliseconds = 1000;

    ROSAction *grab_action = new ROSAction("grab_object_action");
    ROSAction *grab_action2 = new ROSAction("grab_object_action");
    ROSAction *approach_objects_action =
        new ROSAction("approach_objects_action");
    ROSAction *folding_assembly_action =
        new ROSAction("folding_assembly_action");
    ROSAction *insertion_with_deformation_action =
        new ROSAction("insertion_with_deformation_action");
    ROSAction *place_action = new ROSAction("place_action");

    ROSCondition *condition = new ROSCondition("C1");

    // SelectorStarNode *selector1 = new SelectorStarNode("sel1");
    // SequenceStarNode *sequence1 = new SequenceStarNode("seq1");

    SelectorStarNode *selector1 = new SelectorStarNode("sel1");
    SequenceStarNode *sequence1 = new SequenceStarNode("seq1");

    // sequence1->GetType();
    sequence1->AddChild(grab_action);
    sequence1->AddChild(grab_action2);
    sequence1->AddChild(approach_objects_action);
    selector1->AddChild(folding_assembly_action);
    selector1->AddChild(insertion_with_deformation_action);
    // sequence1->AddChild(folding_assembly_action);
    // sequence1->AddChild(insertion_with_deformation_action);
    sequence1->AddChild(selector1);
    sequence1->AddChild(place_action);

    Execute(sequence1, TickPeriod_milliseconds);
  } catch (BehaviorTreeException &Exception) {
    ROS_ERROR("%s", Exception.what());
  }

  return 0;
}
