#ifndef __PLACE_DUMMY__
#define __PLACE_DUMMY__

#include <ros/ros.h>
#include <sarafun_manipulation/PlaceAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestPlace : TestServer<sarafun_manipulation::PlaceAction,
                             sarafun_manipulation::PlaceGoalConstPtr,
                             sarafun_manipulation::PlaceFeedback,
                             sarafun_manipulation::PlaceResult> {
public:
  TestPlace(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_manipulation::PlaceAction,
            sarafun_manipulation::PlaceGoalConstPtr,
            sarafun_manipulation::PlaceFeedback,
            sarafun_manipulation::PlaceResult>::TestServer(node_name,
                                                           actionlib_name) {}

  ~TestPlace() {}

protected:
  bool parseGoal(const sarafun_manipulation::PlaceGoalConstPtr &goal);
};
}

#endif
