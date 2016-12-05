#ifndef __GRASP_DUMMY__
#define __GRASP_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/GraspKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestGrasp : TestServer<sarafun_msgs::GraspKeyframeAction,
                               sarafun_msgs::GraspKeyframeGoalConstPtr,
                               sarafun_msgs::GraspKeyframeFeedback,
                               sarafun_msgs::GraspKeyframeResult> {
public:
  TestGrasp(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::GraspKeyframeAction,
            sarafun_msgs::GraspKeyframeGoalConstPtr,
            sarafun_msgs::GraspKeyframeFeedback,
            sarafun_msgs::GraspKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestGrasp() {}

protected:
  bool parseGoal(const sarafun_msgs::GraspKeyframeGoalConstPtr &goal);
};
}
#endif
