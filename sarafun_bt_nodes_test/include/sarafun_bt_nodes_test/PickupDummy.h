#ifndef __PICKUP_DUMMY__
#define __PICKUP_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/PickUpKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestPickUp : TestServer<sarafun_msgs::PickUpKeyframeAction,
                               sarafun_msgs::PickUpKeyframeGoalConstPtr,
                               sarafun_msgs::PickUpKeyframeFeedback,
                               sarafun_msgs::PickUpKeyframeResult> {
public:
  TestPickUp(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::PickUpKeyframeAction,
            sarafun_msgs::PickUpKeyframeGoalConstPtr,
            sarafun_msgs::PickUpKeyframeFeedback,
            sarafun_msgs::PickUpKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestPickUp() {}

protected:
  bool parseGoal(const sarafun_msgs::PickUpKeyframeGoalConstPtr &goal);
};
}
#endif
