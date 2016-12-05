#ifndef __MOVE_DUMMY__
#define __MOVE_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/MoveKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestMove : TestServer<sarafun_msgs::MoveKeyframeAction,
                               sarafun_msgs::MoveKeyframeGoalConstPtr,
                               sarafun_msgs::MoveKeyframeFeedback,
                               sarafun_msgs::MoveKeyframeResult> {
public:
  TestMove(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::MoveKeyframeAction,
            sarafun_msgs::MoveKeyframeGoalConstPtr,
            sarafun_msgs::MoveKeyframeFeedback,
            sarafun_msgs::MoveKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestMove() {}

protected:
  bool parseGoal(const sarafun_msgs::MoveKeyframeGoalConstPtr &goal);
};
}
#endif
