#ifndef __GRAB_DUMMY__
#define __GRAB_DUMMY__

#include <ros/ros.h>
#include <sarafun_manipulation/GrabAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestGrab : TestServer<sarafun_manipulation::GrabAction,
                            sarafun_manipulation::GrabGoalConstPtr,
                            sarafun_manipulation::GrabFeedback,
                            sarafun_manipulation::GrabResult> {
public:
  TestGrab(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_manipulation::GrabAction,
            sarafun_manipulation::GrabGoalConstPtr,
            sarafun_manipulation::GrabFeedback,
            sarafun_manipulation::GrabResult>::TestServer(node_name,
                                                          actionlib_name) {}

  ~TestGrab() {}

protected:
  bool parseGoal(const sarafun_manipulation::GrabGoalConstPtr &goal);
};
}

#endif
