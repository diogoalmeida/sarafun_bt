#ifndef __APPROACH_DUMMY__
#define __APPROACH_DUMMY__

#include <ros/ros.h>
#include <sarafun_manipulation/ApproachAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestApproach : TestServer<sarafun_manipulation::ApproachAction,
                                sarafun_manipulation::ApproachGoalConstPtr,
                                sarafun_manipulation::ApproachFeedback,
                                sarafun_manipulation::ApproachResult> {
public:
  TestApproach(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_manipulation::ApproachAction,
            sarafun_manipulation::ApproachGoalConstPtr,
            sarafun_manipulation::ApproachFeedback,
            sarafun_manipulation::ApproachResult>::TestServer(node_name,
                                                              actionlib_name) {}

  ~TestApproach() {}

protected:
  bool parseGoal(const sarafun_manipulation::ApproachGoalConstPtr &goal);
};
}
#endif
