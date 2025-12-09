#include "nlp_drone_control/llm_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "llm_planner_node");
  ros::NodeHandle nh;
  ros::MultiThreadedSpinner spinner;
  LLMPlannerNode node(nh);
  spinner.spin();
  return 0;
}

