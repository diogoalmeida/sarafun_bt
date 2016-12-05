#ifndef __CONTACT_DUMMY__
#define __CONTACT_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/ContactKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestContact : TestServer<sarafun_msgs::ContactKeyframeAction,
                               sarafun_msgs::ContactKeyframeGoalConstPtr,
                               sarafun_msgs::ContactKeyframeFeedback,
                               sarafun_msgs::ContactKeyframeResult> {
public:
  TestContact(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::ContactKeyframeAction,
            sarafun_msgs::ContactKeyframeGoalConstPtr,
            sarafun_msgs::ContactKeyframeFeedback,
            sarafun_msgs::ContactKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestContact() {}

protected:
  bool parseGoal(const sarafun_msgs::ContactKeyframeGoalConstPtr &goal);
};
}
#endif
