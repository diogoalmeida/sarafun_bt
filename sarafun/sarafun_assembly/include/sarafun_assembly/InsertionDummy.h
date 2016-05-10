#ifndef __INSERTION_DUMMY__
#define __INSERTION_DUMMY__

#include <ros/ros.h>
#include <sarafun_assembly/InsertionAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestInsertion : TestServer<sarafun_assembly::InsertionAction,
                                 sarafun_assembly::InsertionGoalConstPtr,
                                 sarafun_assembly::InsertionFeedback,
                                 sarafun_assembly::InsertionResult> {
public:
  TestInsertion(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_assembly::InsertionAction,
            sarafun_assembly::InsertionGoalConstPtr,
            sarafun_assembly::InsertionFeedback,
            sarafun_assembly::InsertionResult>::TestServer(node_name,
                                                           actionlib_name) {}

  ~TestInsertion() {}

protected:
  bool parseGoal(const sarafun_assembly::InsertionGoalConstPtr &goal);
};
}
#endif
