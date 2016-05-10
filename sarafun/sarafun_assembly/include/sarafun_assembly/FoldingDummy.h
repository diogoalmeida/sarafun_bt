#ifndef __FOLDING_DUMMY__
#define __FOLDING_DUMMY__

#include <ros/ros.h>
#include <sarafun_assembly/FoldingAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestFolding : TestServer<sarafun_assembly::FoldingAction,
                               sarafun_assembly::FoldingGoalConstPtr,
                               sarafun_assembly::FoldingFeedback,
                               sarafun_assembly::FoldingResult> {
public:
  TestFolding(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_assembly::FoldingAction,
            sarafun_assembly::FoldingGoalConstPtr,
            sarafun_assembly::FoldingFeedback,
            sarafun_assembly::FoldingResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestFolding() {}

protected:
  bool parseGoal(const sarafun_assembly::FoldingGoalConstPtr &goal);
};
}
#endif
