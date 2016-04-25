#include <ros/ros.h>
#include <BehaviorTree.h>
#include <SequenceNode.h>
#include <SelectorNode.h>
#include <Actions/ROSAction.h>
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
    ROSAction *approach_objects_action =
        new ROSAction("approach_objects_action");
    ROSAction *folding_assembly_action =
        new ROSAction("folding_assembly_action");
    ROSAction *insertion_with_deformation_action =
        new ROSAction("insertion_with_deformation_action");
    ROSAction *place_action = new ROSAction("place_action");

    // ROSCondition *condition = new ROSCondition("C1");

    SelectorNode *selector1 = new SelectorNode("sel1");
    SequenceNode *sequence1 = new SequenceNode("seq1");

    // sequence1->addChild(grab_action);
    // sequence1->addChild(grab_action);
    // sequence1->addChild(approach_objects_action);
    // selector1->addChild(folding_assembly_action);
    // selector1->addChild(insertion_with_deformation_action);
    // sequence1->addChild(selector1);
    // sequence1->addChild(place_action);

    Execute(sequence1, TickPeriod_milliseconds);
  } catch (BehaviorTreeException &Exception) {
    std::cout << Exception.what() << std::endl;
  }

  return 0;
}
